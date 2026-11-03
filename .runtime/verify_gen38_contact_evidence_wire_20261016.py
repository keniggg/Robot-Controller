"""Lossless transport receipt for complete archived cuboid proof payloads."""
import pathlib,json,hashlib,sys,time
from alicia_flexible_grasp.grasp.contact_evidence_wire import (
    encode_contact_evidence_wire,decode_contact_evidence_wire,
    LEGACY_MAX_CHARS,WIRE_MAX_CHARS,DECODED_MAX_BYTES)
W=pathlib.Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=D/'verification/gen38_contact_transport_probe_r1'
assert pathlib.Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
canonical=lambda x:json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)
source=D/'verification/gen38_cuboid_silhouette_probe_r1/new_interval_latest.json'
archived=json.loads(source.read_text())['audit']
# This codec-only enclosing object holds the real complete proof. It is not a
# semantically valid contact schema and is never submitted for authorization.
capsule=dict(schema_version=3,codec_roundtrip_only=True,cuboid_rgb_interval=archived)
raw=canonical(capsule);before=raw.encode('utf-8');wire=encode_contact_evidence_wire(raw);after=decode_contact_evidence_wire(wire).encode('utf-8')
assert before==after and json.loads(before)==json.loads(after)
assert archived==json.loads(after)['cuboid_rgb_interval']
envelope=json.loads(wire);assert envelope['raw_json_sha256']==hashlib.sha256(before).hexdigest()
assert envelope['raw_json_bytes']==len(before) and len(wire)<=WIRE_MAX_CHARS
(O/'actual_complete_proof_wire.json').write_text(wire+'\n')
paths=[W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_evidence_wire.py',W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_evidence.py',W/'src/alicia_flexible_grasp_supervisor/tests/test_gen38_contact_evidence_wire.py']
snapshot={str(p):sha(p) for p in paths}
boundary=O/'boundary.jsonl';attempts=boundary.read_text().splitlines() if boundary.exists() else [];assert not attempts
report=dict(passed=True,verified_ns=time.time_ns(),diagnostic_only=True,motion_authorized=False,
    source_path=str(source),source_file_sha256=sha(source),source_stamp_ns=1792207340733134031,
    exact_source_proof_canonical_bytes=len(canonical(archived).encode()),codec_capsule_raw_bytes=len(before),
    encoded_wire_bytes=len(wire.encode()),decoded_bytes=len(after),raw_json_sha256=hashlib.sha256(before).hexdigest(),
    exact_original_json_bytes_preserved=True,complete_original_proof_preserved=True,
    source_proof_contains_all_keys=sorted(archived),
    limits=dict(legacy_max_chars=LEGACY_MAX_CHARS,wire_max_chars=WIRE_MAX_CHARS,decoded_max_bytes=DECODED_MAX_BYTES),
    code_sha256=snapshot,script_sha256=sha(pathlib.Path(__file__)),external_boundary_attempts=attempts,
    guard_scope='Python socket monkeypatch and /dev audit hook; not OS/C isolation',
    source_physical_semantics='Codec-only container preserves complete real proof; it is not an admitted contact-schema object and no physical/contact gate is exercised or bypassed.',
    tests='41 passed in pytest.log; root owns integrated contact/plan/chain verification')
assert snapshot=={str(p):sha(p) for p in paths}
(O/'receipt.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(proof_bytes=report['exact_source_proof_canonical_bytes'],raw_bytes=len(before),wire_bytes=len(wire),decoded_identical=True,receipt_sha256=sha(O/'receipt.json'),code_sha256=snapshot),ensure_ascii=False))
