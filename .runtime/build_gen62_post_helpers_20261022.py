"""Offline deterministic adaptation of the sealed GEN61 post-attempt helpers."""
from pathlib import Path
import hashlib

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
old = W / '.runtime/read_gen61_post_attempt_20261022.py'
new = W / '.runtime/read_gen62_post_attempt_20261022.py'
assert hashlib.sha256(old.read_bytes()).hexdigest() == '29435535f9b5dffe4b7930556b503462cb67b2be814881d92abae69787a7d9a1'
assert not new.exists()
s = old.read_text()

def replace(a, b):
    global s
    assert s.count(a) == 1, (a, s.count(a))
    s = s.replace(a, b)

s = s.replace('GEN61', 'GEN62').replace('gen61_', 'gen62_')
replace("ATTEMPTS = E61 / 'actual_attempt_r1'", "F = E61 / 'contact_feedback_fix_r1'\nATTEMPTS = F / 'actual_attempt_r1'")
replace("RELEASE_SHA = 'f233ba3a41dca9632a13e084b24c5a0c434c70ba0c28f38aeba369ecfb3c17e9'\n", '')
replace("ACCEPTED = E61 / 'runtime_reload_r1/acceptance/result.json'", "ACCEPTED = F / 'runtime_reload_r1/acceptance/result.json'")
replace("ACCEPTED_SHA = '472b99ebf4c6b3a8665a76f5c417e2748a3d09186ab84599ea46a399954c7a86'\n", '')
replace('2b23582e7fbfc868311aecc3fa80ac2cb192cd067c4e187cbdd3eb479356ea0c', '871b1b6b2ae048940beedd9d1a7c214fe2c9ff4201a981e1fd1f71efab6829de')
replace('c4361adae44a5be7d8741882a6408b2ff0a6d383257114ecef408245fbd43080', '7dc87bf19147ebfa24d242203d79ff95542e2f809a0eae9f788c2ddd698146d7')
replace('aa0936f7953d71d22253c9d248d37348446901951d1109e1c932f994e0d4e57f', 'bf12e9a66d663618c3a98b54c63a642549d090a05156499107b64615b795130c')
replace('PINS = {RELEASE: RELEASE_SHA, ACCEPTED: ACCEPTED_SHA, RUNTIME: RUNTIME_SHA,', 'PINS = {RUNTIME: RUNTIME_SHA,')
replace("W / '.runtime/read_gen60_post_attempt_20261022.py':\n        '76dc391ac6617942623e72439eeb12c7a5519ff68b4fd47027ff35616eb379bd'", "W / '.runtime/read_gen61_post_attempt_20261022.py':\n        '29435535f9b5dffe4b7930556b503462cb67b2be814881d92abae69787a7d9a1'")
replace('type(expected_generation) is int and expected_generation == 5', 'type(expected_generation) is int and expected_generation == 6')
replace('this one GEN62 attempt expects generation5', 'this one GEN62 attempt expects generation6')
replace("release_commit['control_revision'] == 67", "release_commit['control_revision'] == 69")
replace("observation['control_revision'] == 67", "observation['control_revision'] == 69")
replace('sha256=ACCEPTED_SHA', 'sha256=PINS[ACCEPTED]')
replace('sha256=RELEASE_SHA', 'sha256=PINS[RELEASE]')
replace('and args.release_sha256 == RELEASE_SHA and args.deployment_sha256 == ACCEPTED_SHA\n        ', '')
replace("supplied = {Path(__file__).resolve(): args.helper_sha256,\n        args.attempt_result.resolve(strict=True): args.attempt_result_sha256,\n        args.runner_exit.resolve(strict=True): args.runner_exit_sha256}\n    PINS.update(supplied)", """supplied = {Path(__file__).resolve(): args.helper_sha256,
        RELEASE: args.release_sha256, ACCEPTED: args.deployment_sha256,
        args.attempt_result.resolve(strict=True): args.attempt_result_sha256,
        args.runner_exit.resolve(strict=True): args.runner_exit_sha256,
        args.authorization.resolve(strict=True): args.authorization_sha256,
        args.record_manifest.resolve(strict=True): args.record_manifest_sha256}
    for path, digest in supplied.items():
        add_pin(path, digest)""")
replace("release['source_count'] == len(release['source_sha256']) == 1101", "release['source_count'] == len(release['source_sha256']) == 1103")
replace("    for path, digest in ctx['pins'].items():\n        if Path(path) in PINS:\n            require(PINS[Path(path)] == digest, 'conflicting runtime input pin')\n        PINS[Path(path)] = digest", "    for path, digest in ctx['pins'].items():\n        add_pin(Path(path), digest)")
replace("    record = Path(entry['record_dir']).resolve(strict=True)\n    require(record.parent == Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records')\n        and exit_path == record / 'user_request_runner_exit.json', 'runner exit not bound to current record')", "    record = closed_record(args, entry, runner_exit)")
replace('    PINS[auth_path] = args.authorization_sha256', '    add_pin(auth_path, args.authorization_sha256)')
replace("    PINS[preflight_path] = entry_inputs[str(preflight_path)]", "    add_pin(preflight_path, entry_inputs[str(preflight_path)])")
replace("    for key in ('release','deployment','attempt-result','runner-exit'):", "    ap.add_argument('--record', type=Path, required=True)\n    for key in ('release','deployment','attempt-result','runner-exit','authorization','record-manifest'):")
replace("    ap.add_argument('--authorization-sha256',required=True)\n", '')
replace("args.expected_generation == 5, 'one exact generation4-to5 successor required'", "args.expected_generation == 6, 'one exact generation5-to6 successor required'")
replace("            selection_authorization=dict(path=str(record/'authorization.json'), sha256=args.authorization_sha256),", "            selection_authorization=dict(path=str(record/'authorization.json'), sha256=args.authorization_sha256),\n            closed_record_manifest=dict(path=str(args.record_manifest.resolve()), sha256=args.record_manifest_sha256),")
insert = '''
def add_pin(path, digest):
    """An explicit caller pin may never replace an existing, different pin."""
    path = Path(path)
    require(type(digest) is str and len(digest) == 64
        and all(c in '0123456789abcdef' for c in digest), 'explicit SHA256 required')
    require(path not in PINS or PINS[path] == digest, 'conflicting post-idle input pin')
    PINS[path] = digest


def closed_record(args, entry, runner_exit):
    """Bind a sealed record, without claiming its result was a physical success."""
    record = args.record.resolve(strict=True)
    require(record.is_dir()
        and record.parent == Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records')
        and Path(entry['record_dir']).resolve(strict=True) == record
        and args.runner_exit.resolve(strict=True) == record / 'user_request_runner_exit.json'
        and args.authorization.resolve(strict=True) == record / 'authorization.json'
        and args.record_manifest.resolve(strict=True) == record / 'manifest.json',
        'entry/exit/authorization/manifest not bound to explicit record')
    for path, digest in ((args.attempt_result, args.attempt_result_sha256),
            (args.runner_exit, args.runner_exit_sha256),
            (args.authorization, args.authorization_sha256),
            (args.record_manifest, args.record_manifest_sha256)):
        require(sha(path) == digest, 'sealed record evidence changed: ' + str(path))
    manifest = read(args.record_manifest)
    require(manifest['schema'] == 1 and manifest['record_id'] == record.name
        and manifest['state'] == 'complete' and manifest['recorder_closed'] is True
        and type(manifest['completed_at']) is str and bool(manifest['completed_at'])
        and type(manifest['result_evidence']['runner_returncode']) is int,
        'record wrapper has not completed and sealed')
    # The recorder wrapper's code is distinct from the original runner's code.
    # Only entry versus runner_exit compare the latter; never conflate them.
    metadata = manifest['metadata']
    command = metadata['command']
    require(metadata['launcher'] == 'run_recorded_grasp.py'
        and type(command) is list and all(type(item) is str for item in command)
        and command.count(str(ENTRY)) == 1, 'record wrapper entry differs')
    flags = {'--release': str(RELEASE), '--release-sha256': args.release_sha256,
        '--deployment': str(ACCEPTED), '--deployment-sha256': args.deployment_sha256,
        '--common-sha256': COMMON_SHA, '--helper-sha256': ENTRY_SHA,
        '--phase': 'recorded', '--scene': str(args.attempt_result.resolve().parent)}
    require(all(command.count(key) == 1 and command.index(key)+1 < len(command)
        and command[command.index(key)+1] == value for key,value in flags.items()),
        'record wrapper command not bound to this scene/deployment')
    auth_rows = [row for row in manifest['files'] if row['path'] == 'authorization.json']
    require(len(auth_rows) == 1 and auth_rows[0]['sha256'] == args.authorization_sha256
        and auth_rows[0]['size_bytes'] == args.authorization.stat().st_size,
        'sealed authorization entry differs')
    return record

'''
replace('\ndef load_bindings(args):', insert + '\ndef load_bindings(args):')
compile(s, str(new), 'exec')
with new.open('x') as f:
    f.write(s)
print(new)
print(hashlib.sha256(new.read_bytes()).hexdigest())
