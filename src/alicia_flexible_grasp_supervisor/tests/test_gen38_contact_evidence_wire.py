"""Lossless transport, legacy compatibility and bounded hostile streams."""
import base64
import hashlib
import json
import zlib

import pytest

from alicia_flexible_grasp.grasp.contact_evidence_wire import (
    LEGACY_MAX_CHARS, WIRE_MAX_CHARS, DECODED_MAX_BYTES,
    encode_contact_evidence_wire, decode_contact_evidence_wire)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def large_content(version=3):
    # Codec-only data, explicitly not a qualified contact model or permission.
    return canonical(dict(schema_version=version, proof=[dict(
        raw_pixel=[i % 653, i % 489], full_original_response=[.123456789, -.987654321],
        source_sha256='fedcba9876543210'*4) for i in range(800)]))


def envelope(raw, compressed=None):
    if isinstance(raw, str):
        raw = raw.encode('utf-8')
    return dict(wire_schema_version=1, encoding='zlib+base64',
        raw_json_bytes=len(raw), raw_json_sha256=hashlib.sha256(raw).hexdigest(),
        payload_base64=base64.b64encode(zlib.compress(raw) if compressed is None else compressed).decode('ascii'))


@pytest.mark.parametrize('version', [1, 2, 3])
def test_small_legacy_json_and_its_exact_bytes_remain_unchanged(version):
    raw = canonical(dict(schema_version=version, finite_bounds=[-.001, .003]))
    assert encode_contact_evidence_wire(raw) == raw
    assert decode_contact_evidence_wire(raw) == raw


def test_large_schema3_roundtrip_preserves_every_original_byte_and_hash():
    raw = large_content(); assert len(raw) > LEGACY_MAX_CHARS
    wire = encode_contact_evidence_wire(raw)
    assert wire == encode_contact_evidence_wire(raw)
    assert len(wire) <= WIRE_MAX_CHARS
    parsed = json.loads(wire)
    assert parsed['raw_json_bytes'] == len(raw.encode())
    assert parsed['raw_json_sha256'] == hashlib.sha256(raw.encode()).hexdigest()
    assert decode_contact_evidence_wire(wire) == raw
    assert json.loads(decode_contact_evidence_wire(wire)) == json.loads(raw)


@pytest.mark.parametrize('version', [1, 2, True, 4])
def test_compression_does_not_expand_old_or_unknown_schema_scope(version):
    raw = large_content(version)
    with pytest.raises(ValueError, match='schema3'):
        encode_contact_evidence_wire(raw)
    with pytest.raises(ValueError, match='schema3'):
        decode_contact_evidence_wire(canonical(envelope(raw)))


def test_large_plain_json_is_rejected_instead_of_bypassing_wire_limits():
    with pytest.raises(ValueError, match='oversized'):
        decode_contact_evidence_wire(large_content())


@pytest.mark.parametrize('mutation', ['unknown', 'version', 'bool_version', 'encoding',
                                     'bool_size', 'negative_size', 'large_size',
                                     'hash', 'uppercase_hash', 'payload_type'])
def test_envelope_has_an_exact_typed_versioned_integrity_contract(mutation):
    value = envelope(large_content())
    if mutation == 'unknown': value['ignored'] = True
    elif mutation == 'version': value['wire_schema_version'] = 2
    elif mutation == 'bool_version': value['wire_schema_version'] = True
    elif mutation == 'encoding': value['encoding'] = 'gzip+base64'
    elif mutation == 'bool_size': value['raw_json_bytes'] = True
    elif mutation == 'negative_size': value['raw_json_bytes'] = -1
    elif mutation == 'large_size': value['raw_json_bytes'] = DECODED_MAX_BYTES+1
    elif mutation == 'hash': value['raw_json_sha256'] = '0'*64
    elif mutation == 'uppercase_hash': value['raw_json_sha256'] = value['raw_json_sha256'].upper()
    else: value['payload_base64'] = ['not', 'text']
    with pytest.raises(ValueError):
        decode_contact_evidence_wire(canonical(value))


@pytest.mark.parametrize('suffix', ['!', '\n', '====', ' '])
def test_base64_must_be_strict_and_canonical(suffix):
    value = envelope(large_content()); value['payload_base64'] += suffix
    with pytest.raises(ValueError, match='base64'):
        decode_contact_evidence_wire(canonical(value))


@pytest.mark.parametrize('mode', ['truncated', 'trailing', 'concatenated', 'wrong_format', 'corrupt'])
def test_zlib_requires_one_complete_stream_and_no_trailing_bytes(mode):
    raw = large_content().encode(); compressed = zlib.compress(raw)
    if mode == 'truncated': compressed = compressed[:-2]
    elif mode == 'trailing': compressed += b'trailing'
    elif mode == 'concatenated': compressed += zlib.compress(b'{}')
    elif mode == 'wrong_format': compressed = b'not-zlib'+compressed
    else: compressed = compressed[:-1]+bytes([compressed[-1] ^ 1])
    with pytest.raises(ValueError, match='zlib'):
        decode_contact_evidence_wire(canonical(envelope(raw, compressed)))


def test_declared_length_mismatch_is_rejected_in_both_directions():
    raw = large_content(); value = envelope(raw)
    for delta in (-1, 1):
        changed = dict(value, raw_json_bytes=value['raw_json_bytes']+delta)
        with pytest.raises(ValueError, match='size'):
            decode_contact_evidence_wire(canonical(changed))


def test_high_expansion_stream_is_stopped_at_hard_decoded_byte_bound():
    raw = b'x'*(DECODED_MAX_BYTES+1)
    value = envelope(raw); value['raw_json_bytes'] = DECODED_MAX_BYTES
    assert len(canonical(value)) < WIRE_MAX_CHARS
    with pytest.raises(ValueError, match='size'):
        decode_contact_evidence_wire(canonical(value))


def test_wire_and_raw_limits_are_enforced_before_unbounded_json_work():
    with pytest.raises(ValueError, match='oversized'):
        decode_contact_evidence_wire(' '* (WIRE_MAX_CHARS+1))
    with pytest.raises(ValueError, match='oversized'):
        encode_contact_evidence_wire(' '* (DECODED_MAX_BYTES+1))


@pytest.mark.parametrize('raw', [
    '{"schema_version":3,"schema_version":3}',
    '{"a":{"x":0,"x":1},"schema_version":3}',
    '{"a":NaN,"schema_version":3}',
    '{"a":Infinity,"schema_version":3}',
    '{"a":1e99999,"schema_version":3}',
    '{ "schema_version":3}',
])
def test_duplicate_nonfinite_and_noncanonical_raw_json_are_never_accepted(raw):
    with pytest.raises(ValueError): encode_contact_evidence_wire(raw)
    with pytest.raises(ValueError): decode_contact_evidence_wire(raw)


def test_outer_json_must_be_canonical_and_have_no_duplicate_fields():
    value = envelope(large_content()); wire = canonical(value)
    with pytest.raises(ValueError, match='canonical'):
        decode_contact_evidence_wire(json.dumps(value, indent=2))
    duplicate = wire[:-1]+',"wire_schema_version":1}'
    with pytest.raises(ValueError, match='duplicate'):
        decode_contact_evidence_wire(duplicate)


def test_inner_json_is_revalidated_after_digest_and_inflate():
    malformed = '{"schema_version":3,"x":'+(' '*17000)+'NaN}'
    with pytest.raises(ValueError, match='nonfinite'):
        decode_contact_evidence_wire(canonical(envelope(malformed)))
    noncanonical = '{ "schema_version":3,"proof":"'+'x'*17000+'"}'
    with pytest.raises(ValueError, match='canonical'):
        decode_contact_evidence_wire(canonical(envelope(noncanonical)))
    with pytest.raises(ValueError, match='UTF8'):
        decode_contact_evidence_wire(canonical(envelope(b'\xff'*17000)))


def test_small_json_cannot_use_an_alternative_compressed_wire_identity():
    with pytest.raises(ValueError, match='large schema3'):
        decode_contact_evidence_wire(canonical(envelope(canonical(dict(schema_version=3)))))


def test_incompressible_proof_does_not_bypass_encoded_wire_bound():
    proof = ''.join(hashlib.sha256(str(i).encode()).hexdigest() for i in range(2500))
    with pytest.raises(ValueError, match='wire payload is oversized'):
        encode_contact_evidence_wire(canonical(dict(schema_version=3, proof=proof)))
