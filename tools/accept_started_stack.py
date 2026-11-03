#!/usr/bin/env python3
"""Run one explicitly revised read-only acceptance of an already started stack.

Original startup/contract/failure bytes remain immutable. The revision may
replace exactly the verifier's current-source pin in memory; it never changes
the startup evidence, configuration, release, authorization or live nodes.
This entry has no dispatch operation and calls only the read-only accept path.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
VERIFIER = W/'tools/full_stack_start.py'


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def verified_bytes(reference):
    raw = Path(reference['path']).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == reference['sha256'], 'revision evidence changed: '+reference['path'])
    return raw


def prepare_revision(revision, revision_reference):
    require(revision.get('schema') == 1 and revision.get('kind') == 'readonly_full_stack_acceptance_revision'
            and revision.get('dispatch_permitted') is False, 'read-only acceptance revision required')
    original = json.loads(verified_bytes(revision['startup_contract']))
    startup = json.loads(verified_bytes(revision['startup_receipt']))
    previous = json.loads(verified_bytes(revision['prior_acceptance']))
    require(startup['contract'] == previous['contract'] == revision['startup_contract']
            and startup.get('dispatched') is True and previous.get('passed') is False,
            'revision must reference the same actually started stack and preserved failure')
    old = revision['previous_verifier_snapshot']; new = revision['current_verifier']
    verified_bytes(old); body = verified_bytes(new)
    require(Path(new['path']).resolve() == VERIFIER
            and original['current_sha256'][str(VERIFIER)] == old['sha256']
            and old['sha256'] != new['sha256'], 'revision must bind original verifier bytes and one reviewed replacement')
    require(revision['entry']['path'] == str(Path(__file__).resolve()), 'read-only entry path differs')
    verified_bytes(revision['entry'])
    output = Path(revision['output_root']).resolve()
    require(output.parent == Path(original['output_root']).resolve()
            and output != Path(revision['prior_acceptance']['path']).resolve().parent,
            'revised acceptance must not overwrite prior evidence')
    effective = copy.deepcopy(original)
    effective['current_sha256'][str(VERIFIER)] = new['sha256']
    module = type(sys)('_revised_full_stack_acceptance')
    module.__file__ = str(VERIFIER)
    exec(compile(body, str(VERIFIER), 'exec'), module.__dict__)
    return module, effective, previous, output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision', type=Path, required=True)
    parser.add_argument('--sha256', required=True)
    args = parser.parse_args()
    reference = dict(path=str(args.revision.resolve(strict=True)), sha256=args.sha256)
    revision = json.loads(verified_bytes(reference))
    module, contract, previous, output = prepare_revision(revision, reference)
    return module.accept(contract, revision['startup_contract'], destination=output,
                         verifier_revision=reference, expected_runtime=previous)


if __name__ == '__main__':
    main()
