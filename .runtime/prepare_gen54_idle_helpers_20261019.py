#!/usr/bin/env python3
"""Create only new GEN54 helper drafts; never import or execute an operational helper."""
import ast
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
M=Path('/media/zhuyupei/SanStand')
E=M/'实验记录/Robot-Controller'
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'
O=V/'gen54_idle_helpers_preparation_r1'
TEMPLATES={
 'gen53_idle_reload_common_20261018.py':'63d5cff576979b44a15de5418341584cbefe3d4f0c577a52f2965448de3e905c',
 'read_gen53_preinstall_readiness_20261018.py':'3016d873e2eb808c9542089bbd35b2c43c6bafd6be94ad466ef8de452df377fe',
 'read_gen53_deployment_preflight_20261018.py':'a3090fdb9d1b8c138b2f1f9b80c8325f8722b7835ea5f6b8fcb4f7d8ee5386ff',
 'load_gen53_idle_compute_gateway_20261018.py':'9c8a649409e22e1f88afa3a1516b8ebdd7c6aa94e46ba26ec25a3580867b940a',
 'finalize_gen53_idle_deployment_20261018.py':'e9cfb2e92a1cbdbebb96fc1e4f5b3e10f55d5a8786db12bd1c8f5760dba3bc29',
 'gen53_observation_measurement_consumers_20261018.launch':'977753d0468b64d48f521de1fbfef8debf07dc451755e83c7b84e31c4aac86bb',
 'start_gen53_observation_measurement_consumers_20261018.sh':'a5027a897c6eff8f77a5272fe80a04132728ab761bf4448b9c149b0cfd31b5f0',
 'install_gen53_validated_candidate_20261018.py':'80a23a975d65ab5188f0851d2a2b8a87eab138ac7b103a97c83e51acb54721fa',
 'seal_gen53_release_20261018.py':'6b5404654c567baef821f52e512a5291e0ff3e3a96f399d43ea99f4b4f64e035',
}
MAPPING={
 "E/'ros_log/full_validated_gen52_restart_20261019T013414Z/gen52_current_runtime_identity.json'":"N/'gen53_runtime/gen53_runtime_deployment.json'",
 'GEN53':'GEN54','gen53':'gen54','GEN52':'GEN53','gen52':'gen53',
 '1065':'1069','320':'322','8967':'9010',
 'dfa045bdf7051cf177e59722ed5ad5b81e856a59e5492d49bc78c2d7a9a367c1':'e9f07af55bfac1ae9fd7562d93f04748b6b2e68139f7bab3e0f645cc46ef255f',
 '75eafaf263889ed8ebb8a018852b2506de7d354d5afe4c282a7fce1f59b6d78c':'d3e8c419d724e7eb941403c2bf6d47e0bc5f4cb77dbb549f93158d6c0d16202f',
 '6609863f28d265d8d9caf1bbba3eaffe3b282538c843c42153ad7095fad195d8':'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748',
 'ac42e374789a6ec26cf25f0076c6e9fb622a8a105e4d231f8f935242a377230f':'3ce8c6f4e6de0be68745042e113f2e727f7b8f3cdc862e4728dca0523b8ebb78',
}
PRODUCTION=[
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py',
 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
]
CONTROL_SHA='135af0f40ab4dbfc30b4d75be7c44c963263c2498484da271f056d8104966833'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def edit(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)


def replace_assignment(text,name,value):
    matches=[n for n in ast.parse(text).body if isinstance(n,ast.Assign)
             and any(isinstance(t,ast.Name) and t.id==name for t in n.targets)]
    assert len(matches)==1
    return edit(text,ast.get_source_segment(text,matches[0]),name+' = '+value)


def main():
    assert M.is_mount() and not os.statvfs(M).f_flag&os.ST_RDONLY
    assert O.parent.stat().st_dev==M.stat().st_dev and not O.exists()
    for name,digest in TEMPLATES.items():assert sha(W/'.runtime'/name)==digest
    dependencies={}
    for p,digest in {
        N/'gen53_runtime/gen53_runtime_deployment.json':MAPPING['75eafaf263889ed8ebb8a018852b2506de7d354d5afe4c282a7fce1f59b6d78c'],
        W/'.runtime/gen54_baseline_source/runner_manifest.json':MAPPING['6609863f28d265d8d9caf1bbba3eaffe3b282538c843c42153ad7095fad195d8'],
        V/'gen53_release_manifest.json':MAPPING['dfa045bdf7051cf177e59722ed5ad5b81e856a59e5492d49bc78c2d7a9a367c1'],
        V/'gen53_full_parallel_r1/pytest.xml':MAPPING['ac42e374789a6ec26cf25f0076c6e9fb622a8a105e4d231f8f935242a377230f'],
        N/'gen54_resume_r1/control_resume_receipt.json':'52adbe70b4871f161243f0adc688e6401caf5388ea8acd22465d000fbad9e830',
        N/'gen54_resume_r1/readonly_idle/result.json':'0574f079f738631a393cb85f918df5dfe2fdbef6630137e4f2a5e5d7241cdcbd',
    }.items():
        assert sha(p)==digest;dependencies[str(p)]=digest
    for relative in ['gen53_idle_helpers_independent_static_review_r1/result.json',
                     'gen53_cold_restart_identity_helper_independent_review_r2/result.json',
                     'gen53_installer_sealer_independent_review_r1/result.json',
                     'gen54_parallel_runner_independent_review_r1/result.json']:
        p=V/relative;assert json.loads(p.read_text())['passed'];dependencies[str(p)]=sha(p)
    resume=json.loads((N/'gen54_resume_r1/control_resume_receipt.json').read_text())
    assert resume['after']['state']=='active' and resume['after']['revision']==56 and resume['control_sha256']==CONTROL_SHA
    idle=json.loads((N/'gen54_resume_r1/readonly_idle/result.json').read_text())
    assert idle['passed'] and idle['manual_mode'] and not idle['task_active']
    assert idle['selection']['generation']==2 and str(idle['selection']['stamp_ns'])=='1792374537080906391'
    pattern=re.compile('|'.join(re.escape(k) for k in sorted(MAPPING,key=len,reverse=True)))
    drafts={}
    for old in TEMPLATES:
        new=old.replace('gen53','gen54');path=W/'.runtime'/new;assert not path.exists()
        text=pattern.sub(lambda m:MAPPING[m[0]],(W/'.runtime'/old).read_text())
        if new in ['gen54_idle_reload_common_20261018.py','read_gen54_preinstall_readiness_20261018.py','install_gen54_validated_candidate_20261018.py']:
            line=next(line for line in text.splitlines() if line.startswith('CONTROL = '))
            text=edit(text,line,line+'\nCONTROL_EXPECTED_SHA = '+repr(CONTROL_SHA))
        if new=='gen54_idle_reload_common_20261018.py':
            text=edit(text,"value['revision'] == 54, 'expected unchanged active control revision 54'",
                "value['revision'] == 56, 'expected unchanged active control revision 56'")
            text=edit(text,"    return value\n\n\nclass Transport", "    require(sha(path) == CONTROL_EXPECTED_SHA, 'control differs from accepted GEN54 resume receipt')\n    return value\n\n\nclass Transport")
        if new in ['read_gen54_preinstall_readiness_20261018.py','install_gen54_validated_candidate_20261018.py']:
            text=edit(text,'assert M.is_mount() and args.expected_control_revision > 0','assert M.is_mount() and args.expected_control_revision == 56')
            text=edit(text,"assert control['revision'] == args.expected_control_revision",
                "assert control['revision'] == args.expected_control_revision == 56\n    assert sha(CONTROL) == CONTROL_EXPECTED_SHA")
        if new=='read_gen54_preinstall_readiness_20261018.py':
            text=edit(text,"assert type(selection['generation']) is int and selection['generation'] == 1",
                "assert type(selection['generation']) is int and selection['generation'] == 2\n        assert str(selection['stamp_ns']) == '1792374537080906391'")
        if new=='read_gen54_deployment_preflight_20261018.py':
            text=edit(text,"selection['generation'] == 1", "selection['generation'] == 2\n            and str(selection.get('stamp_ns')) == '1792374537080906391'")
            text=edit(text,'expected unchanged unknown observe_return generation 1','expected unchanged unknown observe_return generation 2 with exact source stamp')
        if new=='install_gen54_validated_candidate_20261018.py':
            text=replace_assignment(text,'PRODUCTION',repr(set(PRODUCTION)))
            text=replace_assignment(text,'NEW_PRODUCTION','set()\n# Bind only after the final collected candidate inventory is frozen.\nEXPECTED_NEW_TEST_FILES = None')
            text=edit(text,'    args = parser.parse_args()',"    args = parser.parse_args()\n    assert type(EXPECTED_NEW_TEST_FILES) is int and EXPECTED_NEW_TEST_FILES > 0, 'GEN54 candidate test-file inventory is not yet bound'")
            text=edit(text,"assert len(manifest['new_tests']) >= 2","assert len(manifest['new_tests']) == EXPECTED_NEW_TEST_FILES")
            text=edit(text,"for row in manifest['new_production_sources']:","for row in manifest.get('new_production_sources', []):")
            text=edit(text,"assert ready['selection']['generation'] == 1 and ready['selection']['mode'] == 'unknown'",
                "assert ready['selection']['generation'] == 2 and ready['selection']['mode'] == 'unknown'\n    assert str(ready['selection']['stamp_ns']) == '1792374537080906391'")
        if new=='seal_gen54_release_20261018.py':
            text=replace_assignment(text,'SCOPE_EVIDENCE',"None\n# Fail closed until final GEN54 original-failure review and collected inventory are pinned.\nEXPECTED_NEW_TEST_FILES = None")
            text=replace_assignment(text,'PRODUCTION_SOURCES','frozenset('+repr(tuple(PRODUCTION))+')')
            text=edit(text,'    args = parser.parse_args()',"    args = parser.parse_args()\n    require(isinstance(SCOPE_EVIDENCE, dict) and SCOPE_EVIDENCE, 'GEN54 original-failure evidence is not yet bound')\n    require(type(EXPECTED_NEW_TEST_FILES) is int and EXPECTED_NEW_TEST_FILES > 0, 'GEN54 candidate test-file inventory is not yet bound')")
            text=edit(text,'not supplemental_rows and len(new_rows) >= 2','not supplemental_rows and len(new_rows) == EXPECTED_NEW_TEST_FILES')
            text=edit(text,'GEN54 production scope differs from support-plane registration and bounded stationary reacquisition',
                'GEN54 production scope differs from reviewed gripper/contact geometry and candidate materialization')
        drafts[new]=text
    helper_names=['gen54_idle_reload_common_20261018.py','read_gen54_deployment_preflight_20261018.py',
        'load_gen54_idle_compute_gateway_20261018.py','gen54_observation_measurement_consumers_20261018.launch',
        'start_gen54_observation_measurement_consumers_20261018.sh']
    pins={str(W/'.runtime'/name):hashlib.sha256(drafts[name].encode()).hexdigest() for name in helper_names}
    name='finalize_gen54_idle_deployment_20261018.py'
    drafts[name]=replace_assignment(drafts[name],'HELPER_SHA',repr(pins))
    for name,text in drafts.items():
        if name.endswith('.py'):compile(text,str(W/'.runtime'/name),'exec')
        elif name.endswith('.launch'):ET.fromstring(text)
    assert all(sha(W/'.runtime'/name)==digest for name,digest in TEMPLATES.items())
    O.mkdir();files=[]
    for old,old_digest in TEMPLATES.items():
        name=old.replace('gen53','gen54');path=W/'.runtime'/name;text=drafts[name]
        with path.open('x') as f:f.write(text)
        os.chmod(path,(W/'.runtime'/old).stat().st_mode&0o777)
        digest=sha(path);assert digest==hashlib.sha256(text.encode()).hexdigest()
        old_text=(W/'.runtime'/old).read_text()
        diff=''.join(difflib.unified_diff(old_text.splitlines(True),text.splitlines(True),fromfile=str(W/'.runtime'/old),tofile=str(path)))
        (O/(name+'.diff')).write_text(diff)
        files.append(dict(path=str(path),sha256=digest,template=str(W/'.runtime'/old),template_sha256=old_digest))
    shell=W/'.runtime/start_gen54_observation_measurement_consumers_20261018.sh'
    subprocess.run(['bash','-n',str(shell)],check=True)
    assert all(sha(W/'.runtime'/name)==digest for name,digest in TEMPLATES.items())
    result=dict(kind='GEN54_idle_install_seal_reload_helpers_preparation',prepared=True,
        independently_reviewed=False,operational_execution_ready=False,
        previous_deployment=str(N/'gen53_runtime/gen53_runtime_deployment.json'),previous_deployment_sha256=dependencies[str(N/'gen53_runtime/gen53_runtime_deployment.json')],
        expected_control_state='active',expected_control_revision=56,expected_control_sha256=CONTROL_SHA,
        selection=dict(mode='unknown',strategy='observe_return',generation=2,stamp_ns='1792374537080906391'),
        epoch_ns=1792373681250409915,baseline_source_count=1069,baseline_test_files=322,baseline_tests=9010,
        production_allowlist=sorted(PRODUCTION),new_production_allowlist=[],files=files,helper_sha256=pins,
        pending_bindings=['install/seal EXPECTED_NEW_TEST_FILES remains None until actual final collection',
                          'seal SCOPE_EVIDENCE remains None until exact original-failure replay/review receipts',
                          'final candidate manifest/focus/static-review/full execution pins and counts supplied at root execution'],
        new_case_count_guessed=False,new_full_regression_claimed=False,
        preserved_contracts=['4 compute/gateway replaced, 7 hardware/GUI PID/start/parent identities preserved',
          'gateway respawn from original bringup parent only; compute launch include_gateway=false',
          'manual idle, no task/router/action in flight, same selection and epoch, 20 SDK measured feedback samples',
          'live source/helper/control/parameters readback; final fresh measured sample and existing motion delta guard',
          'installed-workspace full regression, exact old and collected new case inventories, all production source imports',
          'control/pointer CAS and last-publication order; source/metadata old evidence unchanged',
          'SanStand mounted writable local_only; upload/delete false; no system-disk fallback'],
        syntax_valid=True,shell_syntax_valid=True,launch_XML_valid=True,helper_main_or_import_executed=False,
        ROS_initialized=False,hardware_contacted=False,temperature_read=False,motion_commands_sent=False,
        lifecycle_commands_sent=False,parameters_changed=False,production_source_modified=False,
        old_helpers_unchanged=True,dependency_sha256=dependencies,generation_mapping=MAPPING,
        preparation_helper=str(Path(__file__).resolve()),preparation_helper_sha256=sha(__file__),created_ns=time.time_ns())
    with (O/'result.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    (O/'summary.md').write_text('已生成 9 份独占 GEN54 安装、封口和 idle reload 草稿，并通过 Python 编译检查、shell -n 与 launch XML 解析；未导入/执行任何运维入口。\n\n绑定实际 GEN53 deployment、GEN53 1,069 源码/322 文件/9,010 用例基线、control active56 精确 SHA、selection generation2/stamp1792374537080906391 与原 epoch。替换4个 compute/gateway、保留7个硬件/GUI及原 gateway parent 的约束保持。\n\ninstaller/sealer 的 EXPECTED_NEW_TEST_FILES 和 sealer SCOPE_EVIDENCE 仍为 None，执行会在任何安装/封口写入前拒绝；待最终候选清单和原失败复核证据固定后绑定并独立复核。未猜测新用例数，未使用当前暂停前的旧 readiness 充当新鲜执行检查。\n')
    print(json.dumps(dict(output=str(O),prepared=True,ready_for_execution=False,result_sha256=sha(O/'result.json'),helper_count=len(files)),ensure_ascii=False))


if __name__=='__main__':main()
