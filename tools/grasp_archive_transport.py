#!/usr/bin/env python3
"""Bounded-memory, resumable GitHub Release asset transport.

The caller owns its durable queue and persists the returned dictionaries. No bag
is copied or compressed: each upload reads a range of the original closed file.
A retry reconstructs the same asset names and verifies existing parts. This
module never decides a grasp result and never removes local recordings.

GitHub documents <2 GiB per asset, 1000 assets per release, and optional SHA-256
asset digests. Keep the default 1 GiB parts. Each record has a base tag;
additional -v002/-v003 releases are created if its assets exceed 1000.
https://docs.github.com/en/rest/releases/assets
https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases
"""

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tempfile
from datetime import datetime, timezone
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode, urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener

CHUNK_SIZE = 1 << 30
IO_SIZE = 1 << 20
API_VERSION = "2026-03-10"


class TransportError(RuntimeError):
    """A retryable transport, integrity or configuration failure (no secrets)."""


class HTTPStatusError(TransportError):
    def __init__(self, status):
        self.status = int(status)
        super().__init__("GitHub returned HTTP {}".format(self.status))


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


def _identity(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def _safe_relative(value):
    value = str(value)
    path = PurePosixPath(value)
    if not value or path.is_absolute() or ".." in path.parts or value in (".", ""):
        raise ValueError("archive paths must be relative file paths without '..'")
    return path.as_posix()


def _asset_name(record_id, relative_path, index, part_sha):
    # A hash of the *original* IDs prevents collisions after filename sanitizing.
    record = re.sub(r"[^a-zA-Z0-9_-]", "-", str(record_id))[:48] or "record"
    record_hash = hashlib.sha256(str(record_id).encode()).hexdigest()[:12]
    path_hash = hashlib.sha256(relative_path.encode()).hexdigest()[:16]
    return "r-{}-{}-f-{}-p{:06d}-{}.part".format(
        record, record_hash, path_hash, index, part_sha[:20])


def plan_file(record_id, source, relative_path, chunk_size=CHUNK_SIZE):
    """Hash a closed regular file once; return ordered deterministic ranges.

    Does not contact GitHub or write any data. At most IO_SIZE bytes are read
    into memory at once, regardless of source size or configured part size.
    """
    if not isinstance(chunk_size, int) or not 0 < chunk_size < 2 * CHUNK_SIZE:
        raise ValueError("chunk_size must be positive and strictly below 2 GiB")
    relative_path = _safe_relative(relative_path)
    source = Path(source)
    before = source.stat()
    if not stat.S_ISREG(before.st_mode) or source.is_symlink():
        raise TransportError("only closed regular files can be archived")
    whole = hashlib.sha256()
    parts = []
    offset = 0
    with source.open("rb") as stream:
        if _identity(os.fstat(stream.fileno())) != _identity(before):
            raise TransportError("source changed while opening")
        while offset < before.st_size or not parts:
            length = min(chunk_size, before.st_size - offset)
            part_hash = hashlib.sha256()
            remaining = length
            while remaining:
                block = stream.read(min(IO_SIZE, remaining))
                if not block:
                    raise TransportError("source was truncated while hashing")
                whole.update(block)
                part_hash.update(block)
                remaining -= len(block)
            digest = part_hash.hexdigest()
            index = len(parts)
            parts.append({"index": index, "offset": offset, "size": length,
                          "sha256": digest,
                          "asset_name": _asset_name(record_id, relative_path, index, digest)})
            offset += length
        if stream.read(1) or _identity(os.fstat(stream.fileno())) != _identity(before):
            raise TransportError("source changed while hashing")
    if _identity(source.stat()) != _identity(before):
        raise TransportError("source replaced while hashing")
    return {"path": relative_path, "size": before.st_size,
            "sha256": whole.hexdigest(), "chunk_size": chunk_size, "parts": parts}


class FileRange:
    """A file-like HTTP request body that cannot read beyond its part."""

    def __init__(self, source, offset, length):
        self.stream = Path(source).open("rb")
        self.stream.seek(offset)
        self.remaining = length
        self.length = length

    def read(self, size=-1):
        if not self.remaining:
            return b""
        # Even a caller asking read(-1) cannot allocate the entire part.
        size = min(IO_SIZE, self.remaining, size if size >= 0 else IO_SIZE)
        block = self.stream.read(size)
        if not block and self.remaining:
            raise TransportError("source truncated during upload")
        self.remaining -= len(block)
        return block

    def close(self):
        self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.close()


class _CredentialSafeRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        target = urlsplit(newurl)
        if target.scheme != "https":
            raise TransportError("refusing a non-HTTPS asset redirect")
        redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
        if redirected is not None and urlsplit(req.full_url).netloc != target.netloc:
            for name in list(redirected.headers):
                if name.lower() == "authorization":
                    redirected.remove_header(name)
        return redirected


def resolve_token():
    """Resolve credentials without ever including their contents in errors."""
    for name in ("GH_TOKEN", "GITHUB_TOKEN"):
        if os.environ.get(name):
            return os.environ[name]
    try:
        result = subprocess.run(["gh", "auth", "token", "--hostname", "github.com"],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                text=True, timeout=20, check=False)
    except (OSError, subprocess.SubprocessError):
        raise TransportError("GitHub credentials unavailable; configure gh auth or GH_TOKEN") from None
    if result.returncode or not result.stdout.strip():
        raise TransportError("GitHub credentials unavailable; configure gh auth or GH_TOKEN")
    return result.stdout.strip()


class GithubReleaseStore:
    def __init__(self, owner, repo, tag, token=None, api_base="https://api.github.com",
                 chunk_size=CHUNK_SIZE, timeout=120, opener=None):
        for value in (owner, repo):
            if not re.fullmatch(r"[A-Za-z0-9_.-]+", str(value)):
                raise ValueError("invalid GitHub owner/repository")
        if not tag:
            raise ValueError("release tag is required")
        if not 0 < chunk_size < 2 * CHUNK_SIZE:
            raise ValueError("chunk_size must be strictly below 2 GiB")
        if urlsplit(api_base).scheme != "https":
            raise ValueError("GitHub API must use HTTPS")
        self.owner, self.repo, self.tag = owner, repo, str(tag)
        self.api_base = api_base.rstrip("/")
        self.chunk_size, self.timeout = chunk_size, timeout
        self._token = token
        self._opener = opener or build_opener(_CredentialSafeRedirect())
        self._release = None

    @property
    def repo_url(self):
        return self.api_base + "/repos/{}/{}".format(self.owner, self.repo)

    def _open(self, method, url, data=None, accept="application/vnd.github+json",
              content_type=None, length=None):
        # Only the configured API and GitHub upload service receive credentials.
        host = urlsplit(url).netloc
        if urlsplit(url).scheme != "https" or host not in {
                urlsplit(self.api_base).netloc, "uploads.github.com"}:
            raise TransportError("refusing credentials to an unexpected asset host")
        if self._token is None:
            self._token = resolve_token()
        headers = {"Accept": accept, "Authorization": "Bearer " + self._token,
                   "X-GitHub-Api-Version": API_VERSION,
                   "User-Agent": "alicia-grasp-archive/1"}
        if content_type:
            headers["Content-Type"] = content_type
        if length is not None:
            headers["Content-Length"] = str(length)
        req = Request(url, data=data, headers=headers, method=method)
        try:
            return self._opener.open(req, timeout=self.timeout)
        except HTTPError as error:
            status = error.code
            if error.fp is not None:
                error.close()
            raise HTTPStatusError(status) from None
        except (URLError, OSError, TimeoutError):
            # Signed redirect URLs and token-bearing requests must not appear in logs.
            raise TransportError("GitHub request failed (network or timeout)") from None

    def _json(self, method, url, body=None):
        data = None if body is None else json.dumps(body).encode("utf-8")
        with self._open(method, url, data=data,
                        content_type="application/json" if data is not None else None,
                        length=len(data) if data is not None else None) as response:
            try:
                return json.load(response)
            except (ValueError, UnicodeError):
                raise TransportError("invalid GitHub JSON response") from None

    def _prepare_tag(self, tag, create=False):
        try:
            return self._json("GET", self.repo_url + "/releases/tags/" + quote(tag, safe=""))
        except HTTPStatusError as error:
            if error.status != 404 or not create:
                raise
            try:
                return self._json("POST", self.repo_url + "/releases", {
                    "tag_name": tag, "name": "Grasp recording " + tag,
                    "body": "Lossless grasp recording archive. See the attached manifest for task result and checksums.",
                    "draft": False, "prerelease": False, "make_latest": "false"})
            except HTTPStatusError as race:
                if race.status != 422:
                    raise
                return self._json("GET", self.repo_url + "/releases/tags/" + quote(tag, safe=""))

    def prepare_release(self, create=False):
        """Look up the exact base tag; create only when explicitly opted in."""
        self._release = self._prepare_tag(self.tag, create=create)
        return dict(self._release)

    def _assets(self, release=None):
        if release is None:
            if self._release is None:
                self.prepare_release(create=False)
            release = self._release
        assets, page = {}, 1
        while True:
            batch = self._json("GET", self.repo_url + "/releases/{}/assets?per_page=100&page={}".format(
                int(release["id"]), page))
            if not isinstance(batch, list):
                raise TransportError("invalid release asset listing")
            for asset in batch:
                if asset["name"] in assets:
                    raise TransportError("duplicate release asset names")
                assets[asset["name"]] = asset
            if len(batch) < 100:
                return assets
            page += 1

    def _volumes(self):
        """Discover consecutive volumes after restart; never infer from local state."""
        if self._release is None:
            self.prepare_release(create=True)
        volumes = [{"tag": self.tag, "release": self._release,
                    "assets": self._assets(self._release)}]
        index = 2
        while True:
            tag = self.tag + "-v{:03d}".format(index)
            try:
                release = self._prepare_tag(tag, create=False)
            except HTTPStatusError as error:
                if error.status == 404:
                    return volumes
                raise
            volumes.append({"tag": tag, "release": release, "assets": self._assets(release)})
            index += 1

    def _next_volume(self, volumes):
        # Existing files are never relocated; a retry finds every deterministic name.
        for volume in reversed(volumes):
            if len(volume["assets"]) < 1000:
                return volume
        tag = self.tag + "-v{:03d}".format(len(volumes) + 1)
        release = self._prepare_tag(tag, create=True)
        volume = {"tag": tag, "release": release, "assets": self._assets(release)}
        volumes.append(volume)
        if len(volume["assets"]) >= 1000:
            return self._next_volume(volumes)
        return volume

    def plan_file(self, record_id, source, relative_path):
        return plan_file(record_id, source, relative_path, self.chunk_size)

    def _get_asset(self, asset_id):
        return self._json("GET", self.repo_url + "/releases/assets/" + str(int(asset_id)))

    def _readback(self, asset_id, writer=None, whole=None, expected_size=None):
        digest, size = hashlib.sha256(), 0
        with self._open("GET", self.repo_url + "/releases/assets/" + str(int(asset_id)),
                        accept="application/octet-stream") as response:
            while True:
                block = response.read(IO_SIZE)
                if not block:
                    break
                digest.update(block)
                size += len(block)
                if expected_size is not None and size > expected_size:
                    raise TransportError("remote streamed readback exceeds expected size")
                if writer is not None:
                    writer.write(block)
                if whole is not None:
                    whole.update(block)
        return size, digest.hexdigest()

    def verify_asset(self, asset, size, sha256, readback=False):
        """Fetch fresh metadata and require size + SHA-256 (or full readback)."""
        fresh = self._get_asset(asset["id"])
        if fresh.get("state") != "uploaded" or int(fresh.get("size", -1)) != size:
            raise TransportError("remote asset is incomplete or has wrong size")
        remote_digest = fresh.get("digest") or ""
        if remote_digest.startswith("sha256:"):
            if remote_digest[7:].lower() != sha256.lower():
                raise TransportError("remote SHA-256 mismatch")
            method = "github_sha256_digest"
        else:
            readback = True
            method = "streamed_readback"
        if readback:
            actual_size, actual_hash = self._readback(fresh["id"], expected_size=size)
            if actual_size != size or actual_hash != sha256:
                raise TransportError("remote streamed readback checksum mismatch")
            method = "streamed_readback"
        return {"asset_id": int(fresh["id"]), "url": fresh.get("url"),
                "download_url": fresh.get("browser_download_url"),
                "verification": method, "verified_at": _utc_now(), "verified": True}

    def upload_file(self, record_id, source, relative_path, progress=None):
        """Upload/reuse verified parts; retry after restart is deterministic.

        `progress(entry)` runs after each verified part, allowing the caller to
        journal progress atomically. The callback receives the current entry.
        Queue-level retry/backoff belongs to the caller; no indefinite sleeps.
        """
        source = Path(source)
        identity = _identity(source.stat())
        entry = self.plan_file(record_id, source, relative_path)
        if _identity(source.stat()) != identity:
            raise TransportError("source changed before upload")
        if self._release is None:
            self.prepare_release(create=True)
        volumes = self._volumes()
        existing = {}
        for volume in volumes:
            for name, asset in volume["assets"].items():
                if name in existing:
                    raise TransportError("duplicate asset name across release volumes")
                existing[name] = (asset, volume)
        entry.update({"repository": self.owner + "/" + self.repo,
                      "release_tag": self.tag, "release_url": self._release.get("html_url"),
                      "upload_status": "uploading"})
        for part in entry["parts"]:
            if _identity(source.stat()) != identity:
                raise TransportError("source changed during upload")
            asset, volume = existing.get(part["asset_name"], (None, None))
            if asset is not None and asset.get("state") == "starter":
                # GitHub can retain the declared Content-Length on a failed
                # upload. Size is not proof of completion. Refresh metadata so
                # a stale listing cannot cause a completed backup to be removed.
                asset = self._get_asset(asset["id"])
                if asset.get("name") != part["asset_name"]:
                    raise TransportError("remote starter asset identity changed")
                if asset.get("state") == "starter":
                    with self._open("DELETE", self.repo_url + "/releases/assets/" + str(int(asset["id"]))):
                        pass
                    del volume["assets"][part["asset_name"]]
                    asset = None
            if asset is None:
                volume = volume or self._next_volume(volumes)
                upload_url = volume["release"]["upload_url"].split("{", 1)[0]
                upload_url += "?" + urlencode({"name": part["asset_name"]})
                try:
                    with FileRange(source, part["offset"], part["size"]) as stream:
                        with self._open("POST", upload_url, data=stream,
                                        content_type="application/octet-stream", length=part["size"]) as response:
                            asset = json.load(response)
                        if stream.remaining:
                            raise TransportError("upload did not consume complete source range")
                except HTTPStatusError as error:
                    if error.status != 422:
                        raise
                    # A concurrent worker or a lost response may have completed it.
                    asset = self._assets(volume["release"]).get(part["asset_name"])
                    if asset is None:
                        raise
                volume["assets"][part["asset_name"]] = asset
                existing[part["asset_name"]] = (asset, volume)
            part.update({"release_tag": volume["tag"],
                         "release_url": volume["release"].get("html_url")})
            part.update(self.verify_asset(asset, part["size"], part["sha256"]))
            if progress:
                progress(entry)
        if _identity(source.stat()) != identity:
            raise TransportError("source changed during upload")
        entry.update({"upload_status": "verified", "verified": True, "verified_at": _utc_now()})
        return entry

    @staticmethod
    def _validate_entry(entry):
        _safe_relative(entry["path"])
        offset = 0
        parts = entry["parts"]
        if not parts:
            raise TransportError("archive entry has no ordered parts")
        for index, part in enumerate(parts):
            if part.get("index") != index or part.get("offset") != offset or part["size"] < 0:
                raise TransportError("archive parts are missing, reordered or noncontiguous")
            if not re.fullmatch(r"[0-9a-f]{64}", part.get("sha256", "")):
                raise TransportError("invalid part checksum")
            offset += part["size"]
        if offset != entry["size"] or not re.fullmatch(r"[0-9a-f]{64}", entry.get("sha256", "")):
            raise TransportError("invalid original file size or checksum")

    def verify_file(self, entry, readback=False):
        """Revalidate remote evidence immediately before retention pruning.

        readback=True also hashes the concatenated remote bytes against the
        original file hash, without creating any local duplicate.
        """
        self._validate_entry(entry)
        if entry.get("repository") != self.owner + "/" + self.repo or entry.get("release_tag") != self.tag:
            raise TransportError("archive belongs to a different repository or release")
        whole = hashlib.sha256()
        for part in entry["parts"]:
            self.verify_asset({"id": part["asset_id"]}, part["size"], part["sha256"])
            if readback:
                size, digest = self._readback(part["asset_id"], whole=whole, expected_size=part["size"])
                if size != part["size"] or digest != part["sha256"]:
                    raise TransportError("remote streamed readback checksum mismatch")
        if readback and whole.hexdigest() != entry["sha256"]:
            raise TransportError("remote original file checksum mismatch")
        return True

    def restore_file(self, entry, destination, progress=None):
        """Stream ordered remote ranges into one new destination and verify it.

        Never overwrites an existing file. The only temporary file *is* the
        restored data; it is renamed once both part and original hashes pass.
        """
        self._validate_entry(entry)
        if entry.get("repository") != self.owner + "/" + self.repo or entry.get("release_tag") != self.tag:
            raise TransportError("archive belongs to a different repository or release")
        destination = Path(destination)
        if destination.exists() or destination.is_symlink():
            raise TransportError("restore destination already exists")
        destination.parent.mkdir(parents=True, exist_ok=True)
        whole, size = hashlib.sha256(), 0
        fd, temporary = tempfile.mkstemp(prefix="." + destination.name + ".restore-", dir=str(destination.parent))
        try:
            with os.fdopen(fd, "wb") as writer:
                for part in entry["parts"]:
                    count, digest = self._readback(part["asset_id"], writer=writer, whole=whole, expected_size=part["size"])
                    if count != part["size"] or digest != part["sha256"]:
                        raise TransportError("restored part checksum mismatch")
                    size += count
                    if progress:
                        progress({"bytes_restored": size, "size": entry["size"]})
                if size != entry["size"] or whole.hexdigest() != entry["sha256"]:
                    raise TransportError("restored original file checksum mismatch")
                writer.flush()
                os.fsync(writer.fileno())
            # link() is atomic and fails if another process created destination.
            os.link(temporary, str(destination))
            os.unlink(temporary)
            directory_fd = os.open(str(destination.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return {"path": str(destination), "size": size, "sha256": whole.hexdigest(), "verified": True}
