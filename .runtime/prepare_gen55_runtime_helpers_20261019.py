"""Prepare reviewable GEN55 idle helpers; never import ROS or deploy on import."""
import ast
import hashlib
import json
from pathlib import Path

W = Path(__file__).resolve().parents[1]
R = W / '.runtime'


def main():
    original = (R/'gen54_idle_reload_common_20261018.py').read_text()
    tree = ast.parse(original)
    names = {'sha', 'read', 'require', 'inside', 'storage', 'write',
             'Transport', 'process', 'exited', 'feedback', 'validate_sources'}
    functions = '\n\n'.join(ast.get_source_segment(original, node)
        for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))
        and node.name in names)
    assert len([n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
                and n.name in names]) == len(names)
    constants = original[original.index('W = Path'):original.index('SUPPLEMENTAL_TESTS =')]
    constants = constants.replace("BASE_SHA = 'e9f07af55bfac1ae9fd7562d93f04748b6b2e68139f7bab3e0f645cc46ef255f'",
        "BASE_SHA = 'ec0d1c1d2ebdb88fc9c77174790190dfff71024e58fea6958b356cb9ebf5d3f8'")
    constants = constants.replace("PREVIOUS = N/'gen53_runtime/gen53_runtime_deployment.json'",
        "PREVIOUS = N/'gen54_runtime/gen54_runtime_deployment.json'")
    constants = constants.replace("PREVIOUS_SHA = 'd3e8c419d724e7eb941403c2bf6d47e0bc5f4cb77dbb549f93158d6c0d16202f'",
        "PREVIOUS_SHA = 'a8d57dd38bdb05988cedbcf8c14d89dc3c12ab076c9d72eac35c4152c5bd16c1'")
    constants = constants.replace("CONTROL_EXPECTED_SHA = '135af0f40ab4dbfc30b4d75be7c44c963263c2498484da271f056d8104966833'",
        "CONTROL_EXPECTED_SHA = 'c9d29dac7b6caadcef967e497fe0bf628a2b6fcec642d48ca13713722020a0f0'")
    for name in ('gen54_idle_reload_common', 'read_gen54_deployment_preflight',
                 'load_gen54_idle_compute_gateway', 'gen54_observation_measurement_consumers',
                 'start_gen54_observation_measurement_consumers'):
        constants = constants.replace(name+'_20261018', name.replace('gen54','gen55')+'_20261019')
    header = '"""GEN55 pinned release and idle reload checks. No ROS on import."""\n'
    imports = 'import hashlib, importlib.util, json, math, os, shutil, time\nfrom pathlib import Path\nimport xmlrpc.client\nfrom collections import Counter\n'
    extra = (R/'gen55_release_validation_body.py').read_text()
    outputs = {'gen55_idle_reload_common_20261019.py': header+imports+constants+'\n'+functions+'\n\n'+extra}
    for filename in ('read_gen54_deployment_preflight_20261018.py',
                     'load_gen54_idle_compute_gateway_20261018.py',
                     'gen54_observation_measurement_consumers_20261018.launch',
                     'start_gen54_observation_measurement_consumers_20261018.sh',
                     'finalize_gen54_idle_deployment_20261018.py'):
        data = (R/filename).read_text()
        # The previous runtime is GEN54; the candidate and helper names are GEN55.
        data = data.replace('GEN54', 'GEN55').replace('gen54', 'gen55').replace('20261018', '20261019')
        data = data.replace('GEN53', 'GEN54').replace('gen53', 'gen54')
        if filename.startswith('read_'):
            data = data.replace("selection['generation'] == 2", "selection['generation'] == 3")
            data = data.replace('1792374537080906391', '1792379927063770294')
            data = data.replace('generation 2', 'generation 3')
        if filename.startswith('finalize_'):
            data = data.replace('full_latest_gen41_restart_20261019T011222Z', 'full_latest_gen41_restart_20261018T011222Z')
            parsed = ast.parse(data)
            node = next(n for n in parsed.body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'HELPER_SHA' for t in n.targets))
            # Bound after the five generated helpers are written below.
            data = data.replace(ast.get_source_segment(data,node), 'HELPER_SHA = __HELPER_SHA__')
        outname = filename.replace('gen54','gen55').replace('20261018','20261019')
        outputs[outname] = data
    helper_names = [n for n in outputs if not n.startswith('finalize_')]
    helper_shas = {str(R/n):hashlib.sha256(outputs[n].encode()).hexdigest() for n in helper_names}
    outputs['finalize_gen55_idle_deployment_20261019.py'] = outputs['finalize_gen55_idle_deployment_20261019.py'].replace('__HELPER_SHA__', repr(helper_shas))
    for name, data in outputs.items():
        path = R/name
        assert not path.exists(), path
        if path.suffix == '.py': ast.parse(data)
        path.write_text(data)
    print(json.dumps({name: hashlib.sha256(data.encode()).hexdigest() for name,data in outputs.items()},indent=2))


if __name__ == '__main__':
    main()
