"""Lossless, bounded transport for already validated contact-evidence JSON.

This codec proves byte integrity and canonical JSON only. The original contact
schema, source, geometry, physical gates and plan signatures remain mandatory.
Small legacy JSON is returned byte-for-byte; only large schema3 JSON is packed.
"""
import base64
import binascii
import hashlib
import json
import math
import zlib


LEGACY_MAX_CHARS = 16384
WIRE_MAX_CHARS = 65536
DECODED_MAX_BYTES = 4*1024*1024
_WIRE_FIELDS = frozenset(('wire_schema_version', 'encoding', 'raw_json_sha256',
                          'raw_json_bytes', 'payload_base64'))
_ENCODING = 'zlib+base64'


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate contact evidence wire JSON field')
        result[key] = value
    return result


def _finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError('nonfinite contact evidence wire JSON number')
    return value


def _bounded_int(text):
    # Timestamps and source dimensions need at most a few dozen digits.
    # Bound parsing work even for a highly compressible integer token.
    if len(text.lstrip('-')) > 128:
        raise ValueError('contact evidence wire JSON integer is oversized')
    return int(text)


def _nonfinite(text):
    raise ValueError('nonfinite contact evidence wire JSON number')


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _parse_canonical(text):
    try:
        parsed = json.loads(text, object_pairs_hook=_unique_object,
            parse_float=_finite_float, parse_int=_bounded_int, parse_constant=_nonfinite)
        if not isinstance(parsed, dict):
            raise ValueError('contact evidence wire JSON must be an object')
        if _canonical(parsed) != text:
            raise ValueError('contact evidence wire JSON is not canonical JSON')
    except (TypeError, OverflowError, RecursionError) as exc:
        raise ValueError('contact evidence wire JSON is malformed or excessive') from exc
    return parsed


def _raw_bytes(text):
    if not isinstance(text, str):
        raise ValueError('contact evidence wire content must be a JSON string')
    # UTF8 bytes can never be shorter than Python's character count.
    if len(text) > DECODED_MAX_BYTES:
        raise ValueError('decoded contact evidence JSON is oversized')
    try:
        raw = text.encode('utf-8')
    except UnicodeError as exc:
        raise ValueError('contact evidence wire JSON is not valid UTF8') from exc
    if len(raw) > DECODED_MAX_BYTES:
        raise ValueError('decoded contact evidence JSON is oversized')
    return raw


def encode_contact_evidence_wire(canonical_json):
    """Pack complete canonical schema3 evidence without dropping any fields.

    The caller must first run its contact-schema and physical validation. This
    helper deliberately does not manufacture or authorize a contact model.
    """
    raw = _raw_bytes(canonical_json)
    parsed = _parse_canonical(canonical_json)
    if len(canonical_json) <= LEGACY_MAX_CHARS:
        return canonical_json
    if type(parsed.get('schema_version')) is not int or parsed['schema_version'] != 3:
        raise ValueError('only large schema3 contact evidence supports compressed transport')
    compressed = zlib.compress(raw, level=9)
    envelope = dict(wire_schema_version=1, encoding=_ENCODING,
        raw_json_sha256=hashlib.sha256(raw).hexdigest(), raw_json_bytes=len(raw),
        payload_base64=base64.b64encode(compressed).decode('ascii'))
    encoded = _canonical(envelope)
    if len(encoded) > WIRE_MAX_CHARS:
        raise ValueError('compressed contact evidence wire payload is oversized')
    return encoded


def decode_contact_evidence_wire(encoded):
    """Return the exact original JSON, with hard wire and inflate bounds.

    A returned JSON string still requires the existing contact decoder's
    schema/source/final-pose/opening validation. The signed wire string remains
    unchanged and must continue to participate in the original plan digest.
    """
    if not isinstance(encoded, str):
        raise ValueError('contact evidence wire payload must be a JSON string')
    if len(encoded) > WIRE_MAX_CHARS:
        raise ValueError('contact evidence wire payload is oversized')
    parsed = _parse_canonical(encoded)
    if 'wire_schema_version' not in parsed:
        if len(encoded) > LEGACY_MAX_CHARS:
            raise ValueError('plain contact geometry evidence is oversized')
        _raw_bytes(encoded)
        return encoded
    if set(parsed) != _WIRE_FIELDS:
        raise ValueError('contact evidence wire envelope fields differ')
    if type(parsed['wire_schema_version']) is not int or parsed['wire_schema_version'] != 1 or parsed['encoding'] != _ENCODING:
        raise ValueError('unsupported contact evidence wire envelope')
    expected_size = parsed['raw_json_bytes']
    digest = parsed['raw_json_sha256']
    if type(expected_size) is not int or not 0 < expected_size <= DECODED_MAX_BYTES:
        raise ValueError('decoded contact evidence declared size is invalid or oversized')
    if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
        raise ValueError('contact evidence wire raw JSON digest is invalid')
    payload = parsed['payload_base64']
    if not isinstance(payload, str):
        raise ValueError('contact evidence wire base64 payload must be text')
    try:
        compressed = base64.b64decode(payload.encode('ascii'), validate=True)
    except (UnicodeError, binascii.Error, ValueError) as exc:
        raise ValueError('contact evidence wire base64 payload is invalid') from exc
    if not compressed or base64.b64encode(compressed).decode('ascii') != payload:
        raise ValueError('contact evidence wire base64 payload is not canonical')
    try:
        stream = zlib.decompressobj(zlib.MAX_WBITS)
        # No unbounded decompress or flush: at most expected_size+1 bytes can
        # be materialized, independent of the compressed expansion ratio.
        raw = stream.decompress(compressed, expected_size+1)
    except zlib.error as exc:
        raise ValueError('contact evidence wire zlib payload is invalid') from exc
    if len(raw) != expected_size:
        raise ValueError('contact evidence wire decoded size differs')
    if not stream.eof or stream.unconsumed_tail or stream.unused_data:
        raise ValueError('contact evidence wire zlib stream is incomplete or has trailing data')
    if hashlib.sha256(raw).hexdigest() != digest:
        raise ValueError('contact evidence wire raw JSON digest differs')
    try:
        decoded = raw.decode('utf-8')
    except UnicodeError as exc:
        raise ValueError('contact evidence wire raw JSON is not valid UTF8') from exc
    content = _parse_canonical(decoded)
    if (len(decoded) <= LEGACY_MAX_CHARS or type(content.get('schema_version')) is not int or
            content['schema_version'] != 3):
        raise ValueError('compressed contact evidence requires large schema3 canonical JSON')
    return decoded
