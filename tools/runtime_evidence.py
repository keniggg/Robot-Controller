"""Verified historical bytes for one orchestration transaction.

This module grants no authorization, freshness, deployment or motion permission.
Callers must explicitly separate sealed historical evidence from current source,
authorization/control, configuration and runtime state. Current inputs still need
their original checks at every boundary; they must never enter this cache.

A transaction retains the exact bytes it verified, not a claim that the original
path is still unchanged. JSON is parsed from those bytes. A new transaction reads
and hashes again, even if filesystem timestamps match. There is no persistent
cache, mtime shortcut, ROS dependency or background service. Transactions are
intended for one orchestration thread and own memory until close().
"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
from types import MappingProxyType
from typing import FrozenSet, Mapping


class EvidenceError(ValueError):
    """An evidence binding or transaction contract was not satisfied."""


def normalize_sha256(*mappings):
    """Merge Path/string bindings without silently replacing a conflicting SHA.

    This normalization is also suitable for uncached current-input maps. It
    performs no content verification and gives no permission to cache an input.
    Pass maps separately instead of merging them with dict.update() first.
    """
    result = {}
    for mapping in mappings:
        for name, digest in mapping.items():
            path = Path(name).resolve()
            if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
                raise EvidenceError("EVIDENCE_SHA256_INVALID: " + str(path))
            if path in result and result[path] != digest:
                raise EvidenceError("EVIDENCE_PATH_CONFLICT: " + str(path))
            result[path] = digest
    return result


@dataclass(frozen=True, init=False)
class ReleaseEvidence:
    """Explicit historical bindings, with known current paths excluded.

    ``current_paths`` is required so the caller declares that boundary. This
    generic class cannot infer whether a receipt is closed or a source is live;
    deciding and validating those facts remains the caller's responsibility.
    """

    historical_sha256: Mapping
    current_paths: FrozenSet[Path]

    def __init__(self, historical_sha256, *, current_paths):
        bindings = normalize_sha256(historical_sha256)
        if isinstance(current_paths, (str, bytes)) or hasattr(current_paths, "__fspath__"):
            raise TypeError("current_paths must be an iterable of paths, not one path")
        current = frozenset(Path(name).resolve() for name in current_paths)
        overlap = current.intersection(bindings)
        if overlap:
            raise EvidenceError("CURRENT_INPUT_CANNOT_BE_CACHED: " + str(sorted(overlap)[0]))
        object.__setattr__(self, "historical_sha256", MappingProxyType(bindings))
        object.__setattr__(self, "current_paths", current)

    def transaction(self):
        return VerificationTransaction(self)


class VerificationTransaction:
    """Read each declared historical path once; invalidate on any read failure."""

    def __init__(self, evidence):
        if not isinstance(evidence, ReleaseEvidence):
            raise TypeError("ReleaseEvidence required")
        self._evidence = evidence
        self._verified = {}
        self._closed = False
        self._read_attempts = 0
        self._bytes_read = 0
        self._hashes = 0
        self._cache_hits = 0

    def _require_open(self):
        if self._closed:
            raise EvidenceError("EVIDENCE_TRANSACTION_CLOSED")

    def read_bytes(self, name):
        self._require_open()
        try:
            path = Path(name).resolve()
            if path not in self._evidence.historical_sha256:
                raise EvidenceError("HISTORICAL_EVIDENCE_NOT_DECLARED: " + str(path))
            if path in self._verified:
                self._cache_hits += 1
                return self._verified[path]
            self._read_attempts += 1
            raw = path.read_bytes()
            self._bytes_read += len(raw)
            self._hashes += 1
            if hashlib.sha256(raw).hexdigest() != self._evidence.historical_sha256[path]:
                raise EvidenceError("HISTORICAL_EVIDENCE_HASH_MISMATCH: " + str(path))
            self._verified[path] = raw
            return raw
        except Exception:
            self.close()
            raise

    def read_json(self, name):
        try:
            # Never reopen the path after verification. Return a new JSON value
            # so a caller cannot mutate the bytes used by later consumers.
            return json.loads(self.read_bytes(name))
        except Exception:
            self.close()
            raise

    def verify_all(self):
        self._require_open()
        for path in self._evidence.historical_sha256:
            self.read_bytes(path)

    @property
    def stats(self):
        """Logical file work, not physical disk traffic or a safety receipt."""
        return dict(read_attempts=self._read_attempts, bytes_read=self._bytes_read,
                    sha256_computations=self._hashes, cache_hits=self._cache_hits)

    def close(self):
        self._closed = True
        self._verified.clear()

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, *_args):
        self.close()
