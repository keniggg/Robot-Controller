"""Bind GEN55 operational helpers to the accepted GEN54 cold restart.

File-only preparation. Never calls ROS, starts/stops a process, installs task
sources, changes control, or publishes the current deployment pointer.
"""
import ast
import difflib
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N/'verification'
C = E/'ros_log/full_validated_gen54_restart_20261020T012700Z'
ACCEPTANCE = C/'cold_restart_acceptance/result.json'
ACCEPTANCE_SHA = '2dbbec8043bf235f7abe039eac7372d22f0ee1e1ddbec0f8dfbe8f462eb548d2'
ADAPTER = C/'gen54_current_runtime_identity.json'
OUT = V/'gen55_cold_restart_binding_r1'
RELEASE = V/'gen54_release_manifest.json'
RELEASE_SHA = 'ec0d1c1d2ebdb88fc9c77174790190dfff71024e58fea6958b356cb9ebf5d3f8'
PRIOR_MANIFEST = V/'gen55_source_installation_r1/installed_candidate.json'
PRIOR_MANIFEST_SHA = '38af05487563d1b19c539fa298bfba09b17b166c9c74dce7035419b7ad96deed'
PRIOR_RESULT = V/'gen55_source_installation_r1/result.json'
PRIOR_RESULT_SHA = '5805ad6a13fd5bffd95fcfac6f4197c583fd50e2d79a7a0c6dd99ba163905dbb'
GATEWAY_RELATIVE = 'core_ros/12936474-cc26-11f1-8561-6b34442c9236/motion_gateway-7.log'
COMPUTE = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
TARGETS = COMPUTE + ('/motion_gateway',)
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
    '/supervisor_camera', '/compliant_gripper_controller', '/grasp_mode_router',
    '/alicia_supervisor_gui')
NAMES = ('gen55_idle_reload_common_20261019.py',
    'read_gen55_deployment_preflight_20261019.py',
    'read_gen55_preinstall_readiness_20261019.py',
    'install_and_seal_gen55_20261019.py',
    'finalize_gen55_idle_deployment_20261019.py',
    'monitor_gen55_actual_handoff_20261019.py',
    'analyze_gen55_actual_resources_20261019.py')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def new_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    require(read(path) == value, 'JSON read-back differs: '+str(path))


def replace(text, old, new):
    require(text.count(old) == 1, 'replacement scope differs: '+old[:120])
    return text.replace(old, new)


def main():
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY,
        'writable SanStand required; no fallback')
    require(C.stat().st_dev == V.stat().st_dev == M.stat().st_dev, 'wrong external filesystem')
    require(not OUT.exists() and not ADAPTER.exists(), 'binding already prepared')
    require(sha(ACCEPTANCE) == ACCEPTANCE_SHA and sha(RELEASE) == RELEASE_SHA,
        'accepted cold restart/release changed')
    cold, release = read(ACCEPTANCE), read(RELEASE)
    require(all(cold.get(k) is True for k in ('deployment_passed', 'whole_patch_deployed',
        'full_restart', 'all_processes_replaced')) and cold['preserved_after'] == {},
        'full cold restart not accepted')
    require(set(cold['after']) == set(TARGETS+PRESERVE)
        and cold['source_manifest_sha256'] == RELEASE_SHA
        and cold['source_sha256'] == release['source_sha256'], 'cold restart source/node scope differs')
    require(cold['epoch_ns'] == 1792459933181800618 and cold['selection'] ==
        dict(mode='unknown', strategy='observe_return', generation=1, stamp_ns='1792459935202315092'),
        'cold epoch/selection differs')
    require(cold['continuation_control_revision'] == 58 and cold['gui_manual'] is True,
        'accepted control/manual mode differs')
    source = release['source_sha256']
    require(len(source) == 1075 and all(sha(W/p) == h for p,h in source.items()),
        'workspace is not exact GEN54 baseline')
    require(not any('.runtime/'+name in source for name in NAMES), 'refusing to edit a sealed source')
    require(sha(PRIOR_MANIFEST) == PRIOR_MANIFEST_SHA and sha(PRIOR_RESULT) == PRIOR_RESULT_SHA,
        'prior installed GEN55 evidence differs')
    prior, installed = read(PRIOR_MANIFEST), read(PRIOR_RESULT)
    added = set(prior['source_sha256'])-set(source)
    require(len(prior['source_sha256']) == 1082 and len(added) == 7
        and installed['source_sha256'] == prior['source_sha256']
        and all(sha(W/p) == prior['source_sha256'][p] for p in added),
        'seven retained additions differ from the prior installed source')
    launch = ET.parse(W/'src/alicia_grasp_modes/launch/perception_modes.launch').getroot()
    nodes = {n.get('name'):n for n in launch.findall('node')}
    require(all(nodes[n[1:]].get('respawn', 'false') == 'false' for n in COMPUTE),
        'old compute consumers must not respawn')
    bringup = ET.parse(W/'src/alicia_flexible_grasp_supervisor/launch/bringup.launch').getroot()
    gateway = [n for n in bringup.findall('node') if n.get('name') == 'motion_gateway']
    require(len(gateway) == 1 and gateway[0].get('respawn') == 'true', 'gateway owner changed')
    require((C/GATEWAY_RELATIVE).is_file(), 'actual gateway log is missing')
    require(all(r['parent_pid'] == int(cold['core_service']['MainPID']) for r in cold['after'].values()),
        'cold-start process parent scope differs')
    original = {name:(W/'.runtime'/name).read_text() for name in NAMES}
    before = {str(W/'.runtime'/name):sha(W/'.runtime'/name) for name in NAMES}
    frozen_cold_helpers = {str(W/'.runtime'/name):sha(W/'.runtime'/name) for name in
        ('restart_validated_full_stack_20261019.py', 'verify_validated_full_restart_20261019.py')}
    adapter = dict(cold)
    adapter.update(after={n:cold['after'][n] for n in TARGETS},
        preserved_after={n:cold['after'][n] for n in PRESERVE},
        cold_restart_all_nodes=cold['after'], cold_restart_acceptance=str(ACCEPTANCE),
        cold_restart_acceptance_sha256=ACCEPTANCE_SHA,
        dedicated_compute_reloader_compatible=True,
        process_groups_semantics='after/preserved_after designate roles for the NEXT four-consumer reload; all eleven nodes were newly created by the accepted cold restart',
        preserved_after_semantics='Current cold-start identities designated for preservation during the NEXT compute/gateway reload; no node was preserved through this full cold restart.',
        adapter_created_ns=time.time_ns(), adapter_preparer=str(Path(__file__).resolve()),
        adapter_preparer_sha256=sha(__file__))
    OUT.mkdir()
    new_json(OUT/'plan.json', dict(actual_acceptance=str(ACCEPTANCE), actual_acceptance_sha256=ACCEPTANCE_SHA,
        baseline_release=str(RELEASE), baseline_release_sha256=RELEASE_SHA,
        new_identity_adapter=str(ADAPTER), old_helper_sha256=before,
        helper_paths_only=list(before), cold_start_helpers_unchanged=frozen_cold_helpers,
        production_sources_modified=False, current_pointer_modified=False,
        backup_before_edit=True, created_ns=time.time_ns()))
    for name, data in original.items():
        p=OUT/'backup_helpers'/name; p.parent.mkdir(exist_ok=True)
        with p.open('x') as stream:
            stream.write(data);stream.flush();os.fsync(stream.fileno())
        require(sha(p) == before[str(W/'.runtime'/name)], 'helper backup differs')
    new_json(ADAPTER, adapter); adapter_sha = sha(ADAPTER)
    changed = dict(original)
    changed[NAMES[0]] = replace(changed[NAMES[0]],
        "PREVIOUS = N/'gen54_runtime/gen54_runtime_deployment.json'",
        "PREVIOUS = E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'")
    changed[NAMES[0]] = replace(changed[NAMES[0]],
        "PREVIOUS_SHA = 'a8d57dd38bdb05988cedbcf8c14d89dc3c12ab076c9d72eac35c4152c5bd16c1'",
        'PREVIOUS_SHA = '+repr(adapter_sha))
    changed[NAMES[1]] = replace(changed[NAMES[1]],
        "type(selection.get('generation')) is int and selection['generation'] == 3\n            and str(selection.get('stamp_ns')) == '1792379927063770294'",
        "type(selection.get('generation')) is int and selection == previous['selection']")
    changed[NAMES[1]] = replace(changed[NAMES[1]],
        'expected unchanged unknown observe_return generation 3 with exact source stamp',
        'expected unchanged accepted cold-start selection with exact source stamp')
    changed[NAMES[1]] = replace(changed[NAMES[1]],
        "epoch == previous['epoch_ns'] == 1792373681250409915", "epoch == previous['epoch_ns']")
    changed[NAMES[2]] = replace(changed[NAMES[2]],
        "DEPLOYMENT = N/'gen54_runtime/gen54_runtime_deployment.json'",
        "DEPLOYMENT = E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'")
    changed[NAMES[2]] = replace(changed[NAMES[2]],
        "DEPLOYMENT_SHA = 'a8d57dd38bdb05988cedbcf8c14d89dc3c12ab076c9d72eac35c4152c5bd16c1'",
        'DEPLOYMENT_SHA = '+repr(adapter_sha))
    changed[NAMES[2]] = replace(changed[NAMES[2]],
        "state['epoch'].stamp.to_nsec() == 1792373681250409915", "state['epoch'].stamp.to_nsec() == deployment['epoch_ns']")
    changed[NAMES[2]] = replace(changed[NAMES[2]],
        "        assert state['epoch'].stamp.to_nsec() == deployment['epoch_ns']\n",
        "        assert state['epoch'].stamp.to_nsec() == deployment['epoch_ns']\n"
        "        assert len({message.header.stamp.to_nsec() for message in history}) >= 20\n"
        "        assert all(message.header.stamp.to_nsec() >= deployment['epoch_ns'] for message in history)\n"
        "        sample_spans = {name:max(message.position[i] for message in history)-\n"
        "            min(message.position[i] for message in history) for i,name in enumerate(joint.name)}\n"
        "        assert all(span <= .004 for span in sample_spans.values())\n")
    changed[NAMES[2]] = replace(changed[NAMES[2]],
        "type(selection['generation']) is int and selection['generation'] == 3\n        assert str(selection['stamp_ns']) == '1792379927063770294'",
        "type(selection['generation']) is int and selection == deployment['selection']")
    changed[NAMES[2]] = replace(changed[NAMES[2]],
        "positions=list(joint.position), sample_count=len(history), epoch_ns=state['epoch'].stamp.to_nsec(),",
        "positions=list(joint.position), sample_count=len(history), epoch_ns=state['epoch'].stamp.to_nsec(),\n"
        "            sample_position_spans=sample_spans, sample_stable_limit=.004,\n"
        "            cold_start_position_used_as_current_reference=False,")
    s = changed[NAMES[3]]
    s = replace(s, "('test_gen54_collision_contract.py', 'test_gen52_contact_terminal_collision_screen.py'))",
        "('test_gen54_collision_contract.py', 'test_gen52_contact_terminal_collision_screen.py',\n     'test_contact_sequence_precheck.py', 'test_gen38_contact_replay_staging.py'))")
    s = replace(s, "READINESS = W/'.runtime/read_gen55_preinstall_readiness_20261019.py'",
        "READINESS = W/'.runtime/read_gen55_preinstall_readiness_20261019.py'\n"
        "PRIOR_INSTALLED_MANIFEST = V/'gen55_source_installation_r1/installed_candidate.json'\n"
        "PRIOR_INSTALLED_MANIFEST_SHA = '"+PRIOR_MANIFEST_SHA+"'\n"
        "PRIOR_INSTALLATION_RESULT = V/'gen55_source_installation_r1/result.json'\n"
        "PRIOR_INSTALLATION_RESULT_SHA = '"+PRIOR_RESULT_SHA+"'")
    s = replace(s, '\ndef verify_review(path, manifest):', '''
def verify_retained_additions(manifest, added):
    require(sha(PRIOR_INSTALLED_MANIFEST) == PRIOR_INSTALLED_MANIFEST_SHA and
        sha(PRIOR_INSTALLATION_RESULT) == PRIOR_INSTALLATION_RESULT_SHA,
        'prior installed GEN55 evidence changed')
    prior, installed = read(PRIOR_INSTALLED_MANIFEST), read(PRIOR_INSTALLATION_RESULT)
    require(installed['passed'] is True and len(prior['source_sha256']) == 1082 and
        installed['source_sha256'] == prior['source_sha256'] and
        Path(installed['installed_manifest']).resolve(strict=True) == PRIOR_INSTALLED_MANIFEST and
        installed['installed_manifest_sha256'] == PRIOR_INSTALLED_MANIFEST_SHA,
        'prior installed source binding differs')
    require(added == {NEW_MODULE}|NEW_TESTS|NEW_FIXTURES and all(
        prior['source_sha256'].get(p) == manifest['source_sha256'][p] and
        (W/p).is_file() and sha(W/p) == manifest['source_sha256'][p] for p in added),
        'retained seven additions differ from prior installed and candidate bytes')
    return sorted(added)


def verify_review(path, manifest):''')
    s = replace(s, "ready['epoch_ns'] == previous['epoch_ns'] and ready['selection'] == dict(mode='unknown',\n        strategy='observe_return',generation=3,stamp_ns='1792379927063770294') and",
        "ready['epoch_ns'] == previous['epoch_ns'] and ready['selection'] == previous['selection'] and")
    s = replace(s, "    require(all(not (W/p).exists() for p in added), 'new source already exists')",
        "    retained = verify_retained_additions(manifest, added)")
    s = replace(s, "        str(READINESS):sha(READINESS)})",
        "        str(READINESS):sha(READINESS), str(PRIOR_INSTALLED_MANIFEST):PRIOR_INSTALLED_MANIFEST_SHA,\n        str(PRIOR_INSTALLATION_RESULT):PRIOR_INSTALLATION_RESULT_SHA})\n    pinned.update({str(W/p):manifest['source_sha256'][p] for p in retained})")
    s = replace(s, "write(output/'plan.json',dict(changed_paths=sorted(changed|added),baseline_sha256=baseline,",
        "write(output/'plan.json',dict(changed_paths=sorted(changed),retained_existing_added_paths=retained,baseline_sha256=baseline,")
    s = replace(s, "        for name in sorted(changed|added):", "        for name in sorted(changed):")
    s = replace(s, "            require(not temporary.exists() and ((name in added and not target.exists()) or\n                (name in changed and sha(target) == baseline[name])), 'source destination changed')",
        "            require(not temporary.exists() and name in changed and\n                sha(target) == baseline[name], 'source destination changed')")
    s = replace(s, "write(output/'result.json',dict(passed=True,source_sha256=context['sources'],installed_paths=installed,",
        "write(output/'result.json',dict(passed=True,source_sha256=context['sources'],installed_paths=installed,\n            retained_existing_added_paths=retained,retained_additions_rewritten=False,")
    changed[NAMES[3]] = s
    s = changed[NAMES[5]]
    s = replace(s, "CURRENT_IDENTITY = ORIGINAL.E/'ros_log/full_validated_gen52_restart_20261019T013414Z/gen52_current_runtime_identity.json'",
        "CURRENT_IDENTITY = ORIGINAL.E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'")
    s = replace(s, "CURRENT_IDENTITY_SHA256 = '75eafaf263889ed8ebb8a018852b2506de7d354d5afe4c282a7fce1f59b6d78c'",
        'CURRENT_IDENTITY_SHA256 = '+repr(adapter_sha))
    s = replace(s, "GATEWAY = CURRENT_IDENTITY.parent/'core_ros/40a36d9c-cb5d-11f1-a61a-77e20f59d0e2/motion_gateway-7.log'",
        'GATEWAY = CURRENT_IDENTITY.parent/'+repr(GATEWAY_RELATIVE))
    changed[NAMES[5]] = s
    changed[NAMES[6]] = replace(changed[NAMES[6]],
        "GATEWAY = E / 'ros_log/full_validated_gen52_restart_20261019T013414Z/core_ros/40a36d9c-cb5d-11f1-a61a-77e20f59d0e2/motion_gateway-7.log'",
        "GATEWAY = E / "+repr('ros_log/'+C.name+'/'+GATEWAY_RELATIVE))
    helper_names = (NAMES[0], NAMES[1], 'load_gen55_idle_compute_gateway_20261019.py',
        'gen55_observation_measurement_consumers_20261019.launch', 'start_gen55_observation_measurement_consumers_20261019.sh')
    helper_sha = {str(W/'.runtime'/n):hashlib.sha256(changed[n].encode()).hexdigest() if n in changed
        else sha(W/'.runtime'/n) for n in helper_names}
    tree = ast.parse(changed[NAMES[4]])
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(t,ast.Name) and t.id == 'HELPER_SHA' for t in node.targets))
    changed[NAMES[4]] = replace(changed[NAMES[4]], ast.get_source_segment(changed[NAMES[4]],assignment),
        'HELPER_SHA = '+repr(helper_sha))
    require(sha(ACCEPTANCE) == ACCEPTANCE_SHA and all(sha(p) == h for p,h in before.items()),
        'input helpers changed during preparation')
    deltas = {}
    for name, data in changed.items():
        ast.parse(data, feature_version=(3,8))
        require(data != original[name], 'unnecessary helper rewrite: '+name)
        path=W/'.runtime'/name; temporary=path.with_name(path.name+'.cold-binding.tmp')
        with temporary.open('x') as stream:
            stream.write(data);stream.flush();os.fsync(stream.fileno())
        os.chmod(temporary,path.stat().st_mode&0o777);os.replace(temporary,path)
        diff=''.join(difflib.unified_diff(original[name].splitlines(True),data.splitlines(True),
            fromfile='before/'+name,tofile='after/'+name))
        (OUT/(name+'.diff')).write_text(diff)
        deltas[str(path)] = dict(before_sha256=before[str(path)],after_sha256=sha(path),
            diff=str(OUT/(name+'.diff')))
    require(all(sha(W/p) == h for p,h in source.items()), 'sealed GEN54 source changed')
    require(all(sha(p) == h for p,h in frozen_cold_helpers.items()) and sha(ACCEPTANCE) == ACCEPTANCE_SHA,
        'cold-start helper or acceptance changed')
    result = dict(passed=True, kind='file_only_GEN55_binding_to_accepted_GEN54_cold_restart',
        adapter=str(ADAPTER),adapter_sha256=adapter_sha,acceptance=str(ACCEPTANCE),acceptance_sha256=ACCEPTANCE_SHA,
        helper_delta=deltas, retained_additions=sorted(added), baseline_source_count=1075,
        baseline_sources_unchanged=True, cold_start_helpers_unchanged=frozen_cold_helpers,
        current_pointer_modified=False, live_ros_contacted=False, hardware_contacted=False,
        task_sources_installed=False, runtime_reloaded=False, future_reload_targets=list(TARGETS),
        future_reload_preserve=list(PRESERVE), all_eleven_new_in_cold_restart=True,
        old_compute_respawn=False, gateway_respawn_by_original_parent=True,
        original_parent_pid=int(cold['core_service']['MainPID']),
        attempt_entrypoint_unchanged=str(W/'.runtime/start_gen55_recorded_attempt_20261019.sh'),
        future_pointer_directory_required=str(C),
        note='Prior helper reviews and historical preparation scripts remain historical; a new delta review is required before install/seal/reload.',
        recorded_ns=time.time_ns())
    new_json(OUT/'result.json',result)
    print(json.dumps(dict(output=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),
        adapter=str(ADAPTER),adapter_sha256=adapter_sha,changed_helpers=len(deltas)),ensure_ascii=False))


if __name__ == '__main__':
    main()
