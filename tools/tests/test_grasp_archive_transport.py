"""Transport tests use a fake GitHub server; no credentials/network/robot I/O."""
import copy
import hashlib
import io
import json
from pathlib import Path
import sys
from urllib.error import HTTPError
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import grasp_archive_transport as transport


class FakeGitHub:
    def __init__(self, digest=True):
        self.release = None
        self.releases = {}
        self.assets = {}
        self.payloads = {}
        self.requests = []
        self.uploads = 0
        self.readbacks = 0
        self.digest = digest
        self.fail_upload = None
        self.lost_response = False
        self.largest_read = 0

    @staticmethod
    def response(value):
        return io.BytesIO(json.dumps(value).encode())

    def open(self, request, timeout=None):
        method, url = request.method, request.full_url
        parts = urlsplit(url)
        path = parts.path
        self.requests.append((method, path))
        if '/releases/tags/' in path:
            tag = path.rsplit('/', 1)[-1]
            if tag not in self.releases:
                raise HTTPError(url, 404, 'not found', {}, None)
            return self.response(self.releases[tag])
        if path.endswith('/releases') and method == 'POST':
            tag = json.loads(request.data)['tag_name']
            release_id = 10 + len(self.releases)
            release = {'id': release_id, 'upload_url': 'https://uploads.github.com/repos/o/r/releases/' + str(release_id) + '/assets{?name,label}',
                       'html_url': 'https://github.com/o/r/releases/tag/' + tag}
            self.releases[tag] = release
            if tag == 'record-1':
                self.release = release
            return self.response(release)
        if '/releases/' in path and path.endswith('/assets') and method == 'GET':
            query = parse_qs(parts.query)
            page = int(query.get('page', ['1'])[0])
            release_id = int(path.rsplit('/', 2)[-2])
            assets = [a for a in self.assets.values() if a.get('release_id', 10) == release_id]
            return self.response(assets[(page-1)*100:page*100])
        if '/releases/' in path and path.endswith('/assets') and method == 'POST':
            name = parse_qs(parts.query)['name'][0]
            if any(x['name'] == name for x in self.assets.values()):
                raise HTTPError(url, 422, 'already exists', {}, None)
            if self.fail_upload is not None and self.uploads == self.fail_upload:
                raise HTTPError(url, 503, 'temporary', {}, None)
            payload = b''
            while True:
                block = request.data.read(8192)
                self.largest_read = max(len(block), self.largest_read)
                if not block:
                    break
                payload += block
            assert len(payload) == int(request.headers['Content-length'])
            asset_id = max(self.assets, default=0) + 1
            asset = {'id': asset_id, 'name': name, 'size': len(payload), 'state': 'uploaded',
                     'release_id': int(path.rsplit('/', 2)[-2]),
                     'url': 'https://api.github.com/repos/o/r/releases/assets/' + str(asset_id),
                     'browser_download_url': 'https://github.com/o/r/releases/download/record-1/' + name}
            if self.digest:
                asset['digest'] = 'sha256:' + hashlib.sha256(payload).hexdigest()
            self.assets[asset_id] = asset
            self.payloads[asset_id] = payload
            self.uploads += 1
            if self.lost_response:
                self.lost_response = False
                raise HTTPError(url, 503, 'response lost', {}, None)
            return self.response(asset)
        if '/releases/assets/' in path:
            asset_id = int(path.rsplit('/', 1)[-1])
            if method == 'DELETE':
                del self.assets[asset_id]
                self.payloads.pop(asset_id, None)
                return io.BytesIO()
            if asset_id not in self.assets:
                raise HTTPError(url, 404, 'missing', {}, None)
            if request.headers.get('Accept') == 'application/octet-stream':
                self.readbacks += 1
                return io.BytesIO(self.payloads[asset_id])
            return self.response(self.assets[asset_id])
        raise AssertionError((method, path))


def make_store(server, chunk_size=5):
    return transport.GithubReleaseStore('o', 'r', 'record-1', token='never-print-me',
                                         chunk_size=chunk_size, opener=server)


@pytest.fixture
def source(tmp_path):
    source = tmp_path / 'original.bag'
    source.write_bytes(b'0123456789abcdefghijk')
    return source


def test_plan_order_hashes_names_and_no_extra_file(source):
    before = set(source.parent.iterdir())
    entry = transport.plan_file('record:1', source, 'logs/original.bag', chunk_size=5)
    assert entry['size'] == 21
    assert entry['sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert [part['offset'] for part in entry['parts']] == [0, 5, 10, 15, 20]
    assert [part['size'] for part in entry['parts']] == [5, 5, 5, 5, 1]
    assert entry == transport.plan_file('record:1', source, 'logs/original.bag', 5)
    other = transport.plan_file('record/1', source, 'logs/original.bag', 5)
    assert entry['parts'][0]['asset_name'] != other['parts'][0]['asset_name']
    assert set(source.parent.iterdir()) == before
    for part in entry['parts']:
        data = source.read_bytes()[part['offset']:part['offset'] + part['size']]
        assert hashlib.sha256(data).hexdigest() == part['sha256']


@pytest.mark.parametrize('name', ['/absolute', '../parent', 'a/../../b', '.', ''])
def test_unsafe_relative_name_rejected(source, name):
    with pytest.raises(ValueError):
        transport.plan_file('r', source, name, 4)


@pytest.mark.parametrize('size', [0, -1, 2 << 30, 3 << 30])
def test_invalid_chunk_size(source, size):
    with pytest.raises(ValueError):
        transport.plan_file('r', source, 'x.bag', size)


def test_symlink_input_rejected(source):
    link = source.parent / 'link'
    link.symlink_to(source)
    with pytest.raises(transport.TransportError, match='regular'):
        transport.plan_file('r', link, 'x.bag')


def test_range_reader_never_reads_past_range_or_buffer(source, monkeypatch):
    monkeypatch.setattr(transport, 'IO_SIZE', 3)
    with transport.FileRange(source, 2, 8) as stream:
        assert stream.read() == b'234'
        assert stream.read(100) == b'567'
        assert stream.read(1) == b'8'
        assert stream.read(10) == b'9'
        assert stream.read() == b''


def test_prepare_release_dry_read_never_creates():
    server = FakeGitHub()
    with pytest.raises(transport.HTTPStatusError):
        make_store(server).prepare_release()
    assert not any(method == 'POST' for method, _ in server.requests)


def test_upload_verify_restore_with_real_chunk_reassembly(source):
    server = FakeGitHub()
    store = make_store(server)
    callbacks = []
    entry = store.upload_file('record-1', source, 'original.bag', progress=lambda entry: callbacks.append(copy.deepcopy(entry)))
    assert entry['upload_status'] == 'verified'
    assert len(callbacks) == 5
    assert sum('asset_id' in p for p in callbacks[0]['parts']) == 1
    assert server.uploads == 5
    assert server.readbacks == 0
    assert store.verify_file(entry)
    assert store.verify_file(entry, readback=True)
    output = source.parent / 'restored' / 'original.bag'
    assert store.restore_file(entry, output)['verified']
    assert output.read_bytes() == source.read_bytes()
    assert not list(output.parent.glob('*.restore-*'))
    assert server.readbacks == 10


def test_resume_after_partial_upload_creates_only_missing_parts(source):
    server = FakeGitHub()
    server.fail_upload = 2
    with pytest.raises(transport.HTTPStatusError):
        make_store(server).upload_file('record-1', source, 'original.bag')
    assert server.uploads == 2
    server.fail_upload = None
    entry = make_store(server).upload_file('record-1', source, 'original.bag')
    assert server.uploads == 5
    assert entry['verified']
    make_store(server).upload_file('record-1', source, 'original.bag')
    assert server.uploads == 5


def test_resume_after_response_lost_reuses_verified_asset(source):
    server = FakeGitHub()
    server.lost_response = True
    with pytest.raises(transport.HTTPStatusError):
        make_store(server).upload_file('record-1', source, 'original.bag')
    assert server.uploads == 1
    assert make_store(server).upload_file('record-1', source, 'original.bag')['verified']
    assert server.uploads == 5


def test_missing_server_digest_requires_streamed_readback(source):
    server = FakeGitHub(digest=False)
    entry = make_store(server).upload_file('record-1', source, 'original.bag')
    assert server.readbacks == 5
    assert all(p['verification'] == 'streamed_readback' for p in entry['parts'])


def test_changed_remote_digest_blocks_verification_and_reuse(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    server.assets[1]['digest'] = 'sha256:' + '0' * 64
    with pytest.raises(transport.TransportError, match='SHA-256 mismatch'):
        store.verify_file(entry)
    with pytest.raises(transport.TransportError, match='SHA-256 mismatch'):
        make_store(server).upload_file('record-1', source, 'original.bag')
    assert server.uploads == 5


def test_readback_corruption_blocks_restore_without_publishing_destination(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    server.payloads[2] = b'bad!!'
    dest = source.parent / 'damaged.bag'
    with pytest.raises(transport.TransportError, match='checksum mismatch'):
        store.restore_file(entry, dest)
    assert not dest.exists()
    assert not list(source.parent.glob('.damaged.bag.restore-*'))
    assert source.exists()
    with pytest.raises(transport.TransportError, match='checksum mismatch'):
        store.verify_file(entry, readback=True)


def test_original_hash_mismatch_blocks_reassembly(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    entry['sha256'] = '0' * 64
    with pytest.raises(transport.TransportError, match='original file checksum'):
        store.restore_file(entry, source.parent / 'bad.bag')
    with pytest.raises(transport.TransportError, match='original file checksum'):
        store.verify_file(entry, readback=True)


@pytest.mark.parametrize('change', ['order', 'offset', 'size', 'empty'])
def test_invalid_part_sequence_rejected_before_network(source, change):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    if change == 'order':
        entry['parts'] = entry['parts'][::-1]
    elif change == 'offset':
        entry['parts'][1]['offset'] += 1
    elif change == 'size':
        entry['size'] += 1
    else:
        entry['parts'] = []
    count = len(server.requests)
    with pytest.raises(transport.TransportError):
        store.restore_file(entry, source.parent / 'bad.bag')
    assert len(server.requests) == count


def test_does_not_overwrite_existing_destination(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    with pytest.raises(transport.TransportError, match='already exists'):
        store.restore_file(entry, source)
    assert source.read_bytes() == b'0123456789abcdefghijk'


def test_empty_file_roundtrip(tmp_path):
    original = tmp_path / 'empty.log'
    original.touch()
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', original, 'empty.log')
    assert len(entry['parts']) == 1
    assert entry['parts'][0]['size'] == 0
    store.restore_file(entry, tmp_path / 'restored.log')
    assert (tmp_path / 'restored.log').read_bytes() == b''


def test_source_changed_in_upload_callback_fails(source):
    server = FakeGitHub()
    def mutate(entry):
        source.write_bytes(source.read_bytes() + b'changed')
    with pytest.raises(transport.TransportError, match='source changed'):
        make_store(server).upload_file('record-1', source, 'original.bag', progress=mutate)
    assert server.uploads == 1


def test_missing_remote_part_blocks_verify(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    del server.assets[2]
    with pytest.raises(transport.HTTPStatusError):
        store.verify_file(entry)


def test_cross_host_redirect_strips_authorization():
    handler = transport._CredentialSafeRedirect()
    req = Request('https://api.github.com/file', headers={'Authorization': 'Bearer secret'})
    redirected = handler.redirect_request(req, None, 302, 'found', {}, 'https://release-assets.githubusercontent.com/file?signature=abc')
    assert not redirected.has_header('Authorization')
    same = handler.redirect_request(req, None, 302, 'found', {}, 'https://api.github.com/another')
    assert same.get_header('Authorization') == 'Bearer secret'
    with pytest.raises(transport.TransportError, match='non-HTTPS'):
        handler.redirect_request(req, None, 302, 'found', {}, 'http://bad.example/file')


def test_unexpected_upload_host_never_receives_credentials():
    server = FakeGitHub()
    store = make_store(server)
    with pytest.raises(transport.TransportError, match='unexpected'):
        store._open('POST', 'https://bad.example/file', data=b'secret')
    assert server.requests == []


def test_credentials_never_in_transport_exception(source):
    server = FakeGitHub()
    server.fail_upload = 0
    with pytest.raises(transport.TransportError) as caught:
        make_store(server).upload_file('record-1', source, 'original.bag')
    assert 'never-print-me' not in str(caught.value)


def test_file_range_truncation_detected(source):
    with transport.FileRange(source, 20, 3) as stream:
        assert stream.read() == b'k'
        with pytest.raises(transport.TransportError, match='truncated'):
            stream.read()


def test_release_asset_pagination(source):
    server = FakeGitHub()
    store = make_store(server)
    store.prepare_release(create=True)
    for index in range(205):
        server.assets[1000+index] = {'id': 1000+index, 'name': 'existing-' + str(index)}
    assert len(store._assets()) == 205
    assert sum(path.endswith('/releases/10/assets') for method, path in server.requests) == 3


def test_rollover_resumes_across_release_asset_limit(source):
    server = FakeGitHub()
    store = make_store(server)
    store.prepare_release(create=True)
    for index in range(999):
        server.assets[index+1] = {'id': index+1, 'name': 'older-' + str(index), 'release_id': 10}
    server.fail_upload = 2
    with pytest.raises(transport.HTTPStatusError):
        store.upload_file('record-1', source, 'original.bag')
    assert set(server.releases) == {'record-1', 'record-1-v002'}
    assert server.uploads == 2
    server.fail_upload = None
    restarted = make_store(server)
    entry = restarted.upload_file('record-1', source, 'original.bag')
    assert server.uploads == 5
    assert entry['release_tag'] == 'record-1'
    assert [part['release_tag'] for part in entry['parts']] == ['record-1'] + ['record-1-v002'] * 4
    assert sum(a['release_id'] == 10 for a in server.assets.values()) == 1000
    assert sum(a['release_id'] == 11 for a in server.assets.values()) == 4
    assert restarted.verify_file(entry, readback=True)
    restored = source.parent / 'volumes-restored.bag'
    restarted.restore_file(entry, restored)
    assert restored.read_bytes() == source.read_bytes()
    make_store(server).upload_file('record-1', source, 'original.bag')
    assert server.uploads == 5


def test_single_file_can_exceed_1000_parts_without_changed_capture(tmp_path):
    source = tmp_path / 'large-number-of-parts.bag'
    source.write_bytes(b'x' * 1005)
    server = FakeGitHub()
    store = make_store(server, chunk_size=1)
    entry = store.upload_file('record-1', source, source.name)
    assert len(entry['parts']) == 1005
    assert entry['parts'][999]['release_tag'] == 'record-1'
    assert entry['parts'][1000]['release_tag'] == 'record-1-v002'
    assert len(server.releases) == 2
    assert store.verify_file(entry, readback=True)


@pytest.mark.parametrize("declared_size", [0, 5, 1 << 30])
def test_starter_artifact_only_is_removed_before_retry(source, declared_size):
    server = FakeGitHub()
    store = make_store(server)
    store.prepare_release(create=True)
    plan = store.plan_file('record-1', source, 'original.bag')
    server.assets[99] = {'id': 99, 'name': plan['parts'][0]['asset_name'], 'state': 'starter', 'size': declared_size}
    entry = store.upload_file('record-1', source, 'original.bag')
    assert entry['verified']
    assert [('DELETE', '/repos/o/r/releases/assets/99')] == [x for x in server.requests if x[0] == 'DELETE']
    assert source.exists()


def test_uploaded_asset_never_deleted_on_size_mismatch(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    server.assets[1]['size'] += 1
    with pytest.raises(transport.TransportError, match='wrong size'):
        store.verify_file(entry)
    assert not any(method == 'DELETE' for method, path in server.requests)


def test_restore_refuses_foreign_record_and_excess_remote_bytes(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    foreign = copy.deepcopy(entry)
    foreign['release_tag'] = 'unrelated'
    with pytest.raises(transport.TransportError, match='different repository or release'):
        store.restore_file(foreign, source.parent / 'foreign.bag')
    server.payloads[1] += b'excess'
    with pytest.raises(transport.TransportError, match='exceeds expected size'):
        store.restore_file(entry, source.parent / 'overflow.bag')
    assert not (source.parent / 'overflow.bag').exists()


def test_stale_starter_listing_does_not_delete_completed_upload(source):
    server = FakeGitHub()
    store = make_store(server)
    entry = store.upload_file('record-1', source, 'original.bag')
    original_assets = store._assets

    def stale_listing(release=None):
        assets = original_assets(release)
        assets[entry['parts'][0]['asset_name']]['state'] = 'starter'
        return assets

    store._assets = stale_listing
    assert store.upload_file('record-1', source, 'original.bag')['verified']
    assert server.uploads == 5
    assert not any(method == 'DELETE' for method, path in server.requests)
