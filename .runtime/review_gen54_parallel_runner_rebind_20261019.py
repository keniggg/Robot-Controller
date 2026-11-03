#!/usr/bin/env python3
"""Independent file-only GEN54 runner generation/baseline rebind review."""
import ast
from collections import Counter
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
M=Path('/media/zhuyupei/SanStand')
E=M/'实验记录/Robot-Controller'
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
O=V/'gen54_parallel_runner_independent_review_r1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def inventory(path):
    tree=ET.parse(path).getroot()
    summary={key:sum(int(s.get(key,0)) for s in tree.iter('testsuite'))
             for key in ['tests','failures','errors','skipped']}
    cases=[(c.get('file',''),c.get('classname',''),c.get('name','')) for c in tree.iter('testcase')]
    assert len(cases)==summary['tests']==len(set(cases))
    for key,tag in [('failures','failure'),('errors','error'),('skipped','skipped')]:
        assert summary[key]==sum(len(c.findall(tag)) for c in tree.iter('testcase'))
    return summary,cases


def main():
    started=time.monotonic();cpu_started=time.process_time()
    assert M.is_mount() and O.parent.stat().st_dev==M.stat().st_dev
    assert not os.statvfs(M).f_flag&os.ST_RDONLY and not O.exists()
    pins={
      W/'.runtime/verify_gen53_parallel_offline_20261018.py':'a8e9f967159df358edaba538f7a04b14bf04ada7f99e38640ab96038223e5c84',
      W/'.runtime/verify_gen54_parallel_offline_20261018.py':'f98f940c6f23c3d24205845a00f2c6feaa8ce418b27c9a95b3f6117126ed5c18',
      W/'.runtime/gen54_baseline_source/runner_manifest.json':'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748',
      V/'gen53_full_parallel_r1/result.json':'13a428ac36f300a18a80adb00603c09f6ee70c90342d6a847afeac125883ff8a',
      V/'gen53_full_parallel_r1/pytest.xml':'3ce8c6f4e6de0be68745042e113f2e727f7b8f3cdc862e4728dca0523b8ebb78',
      V/'gen53_release_manifest.json':'e9f07af55bfac1ae9fd7562d93f04748b6b2e68139f7bab3e0f645cc46ef255f',
      W/'.runtime/verify_gen43_offline_tests_20261018_r2.py':'de1b3727a78644ade82690dbfd748745c4e03969eda18556bd1192b54c55698f',
      W/'.runtime/verify_gen43_two_process_draft_20261018.py':'82c6efbf6947577c561ea7427d4dd8fd346b16f1a7ee0f5302d384751564be3b',
      V/'gen43_two_process_pilot_monitor_r1/verification.json':'e8361d8a905351de2be64cd259008c3bc97193f4a0ec1cb16bf26306ceb3066b',
      W/'.runtime/verify_gen41_offline_tests_20261017.py':'0c7c13321f4eed998d54f3ff49456179c7c4c28deb193e9ab646113554de21c9',
      W/'.runtime/verify_gen42_offline_tests_20261017.py':'226b7f0cb282be162ea3ccd5c6e236e4127a8e6b3d9f0769772169962e383342',
    }
    for p,digest in pins.items():assert sha(p)==digest,str(p)
    prep=read(V/'gen54_parallel_runner_preparation_r1/result.json')
    old_path=Path(prep['source']);new_path=Path(prep['draft'])
    assert prep['source_sha256']==pins[old_path] and prep['draft_sha256']==pins[new_path]
    old=old_path.read_text();new=new_path.read_text()
    mapping=prep['mapping']
    expected_keys={'GEN53','gen53','GEN52','gen52','1065','320','8967',
        'a3cf8d431caa7c932aad44e4dd367b02537398352387100efe55bd7ab1d5b5e4',
        '6609863f28d265d8d9caf1bbba3eaffe3b282538c843c42153ad7095fad195d8',
        'ac42e374789a6ec26cf25f0076c6e9fb622a8a105e4d231f8f935242a377230f',
        'dfa045bdf7051cf177e59722ed5ad5b81e856a59e5492d49bc78c2d7a9a367c1'}
    assert set(mapping)==expected_keys
    assert {k:mapping[k] for k in ['GEN53','gen53','GEN52','gen52','1065','320','8967']}=={
        'GEN53':'GEN54','gen53':'gen54','GEN52':'GEN53','gen52':'gen53','1065':'1069','320':'322','8967':'9010'}
    pattern=re.compile('|'.join(re.escape(k) for k in sorted(mapping,key=len,reverse=True)))
    transformed=pattern.sub(lambda m:mapping[m[0]],old)
    assert transformed==new,'unexplained textual change outside approved rebind'
    compile(new,str(new_path),'exec')
    assert ast.dump(ast.parse(transformed),include_attributes=False)==ast.dump(ast.parse(new),include_attributes=False)
    old_functions={n.name:ast.get_source_segment(old,n) for n in ast.parse(old).body if isinstance(n,ast.FunctionDef)}
    new_functions={n.name:ast.get_source_segment(new,n) for n in ast.parse(new).body if isinstance(n,ast.FunctionDef)}
    assert old_functions.keys()==new_functions.keys()
    for name,body in old_functions.items():assert pattern.sub(lambda m:mapping[m[0]],body)==new_functions[name]
    unchanged_functions=[k for k,v in old_functions.items() if v==new_functions[k]]
    bpath=W/'.runtime/gen54_baseline_source/runner_manifest.json';baseline=read(bpath)
    history=read(V/'gen53_full_parallel_r1/result.json');release=read(V/'gen53_release_manifest.json')
    assert baseline['kind']=='GEN54_exact_GEN53_baseline_source' and baseline['baseline_generation']=='GEN53'
    assert Path(baseline['original_full_regression'])==V/'gen53_full_parallel_r1/result.json'
    assert baseline['original_full_regression_sha256']==pins[V/'gen53_full_parallel_r1/result.json']
    assert Path(baseline['baseline_release'])==V/'gen53_release_manifest.json'
    assert baseline['baseline_release_sha256']==pins[V/'gen53_release_manifest.json']
    assert baseline['source_sha256']==history['source_sha256']==release['source_sha256']
    assert baseline['source_count']==history['source_count']==release['source_count']==1069
    assert len(baseline['source_sha256'])==1069
    assert Path(baseline['workspace_root']).resolve()==bpath.parent.resolve()
    checked=[]
    for relative,digest in baseline['source_sha256'].items():
        assert not Path(relative).is_absolute() and '..' not in Path(relative).parts
        path=bpath.parent/relative;actual=sha(path);assert actual==digest,relative
        checked.append(dict(path=relative,sha256=actual,bytes=path.stat().st_size))
    assert history['passed'] and history['full_regression_passed'] and history['scope_complete'] and history['source_unchanged']
    assert history['full_regression_scope'] and history['mode']=='full' and history['worker_count']==7
    assert history['runner_sha256']==pins[old_path] and release['runner_sha256']==pins[old_path]
    assert not history['missing_worker_results'] and not history['duplicate_case_identities']
    assert not history['missing_case_identities'] and not history['unexpected_case_identities']
    assert not history['external_boundary_attempts'] and not history['foreign_or_inconsistent_imports']
    summary,cases=inventory(V/'gen53_full_parallel_r1/pytest.xml');counts=dict(Counter(c[0] for c in cases))
    assert summary==baseline['test_summary']==history['summary']==release['test_summary']==dict(tests=9010,failures=0,errors=0,skipped=0)
    assert counts==baseline['test_case_counts_by_file']==history['test_case_counts_by_file']
    assert len(counts)==len(baseline['test_scope'])==len(set(baseline['test_scope']))==322
    assert set(counts)==set(baseline['test_scope']) and set(counts)<=set(baseline['source_sha256'])
    assert history['test_scope_files']==baseline['test_scope_files']==release['test_scope_files']==322
    worker_cases=[];worker_refs=[]
    assert len(history['worker_result_sha256'])==7
    for name,digest in history['worker_result_sha256'].items():
        path=Path(name);assert sha(path)==digest
        result=read(path);assert result['passed'] and result['source_unchanged'] and result['scope_complete']
        xml=path.parent/'pytest.xml';assert sha(xml)==result['xml_sha256']
        child_summary,child_cases=inventory(xml);assert child_summary==result['summary']
        assert [list(c) for c in child_cases]==result['case_identities']
        worker_cases+=child_cases;worker_refs.append(dict(path=str(path),sha256=digest,xml_sha256=result['xml_sha256'],tests=len(child_cases)))
    assert len(worker_cases)==len(set(worker_cases))==9010 and sorted(worker_cases)==sorted(cases)
    guard_tree=ast.parse((W/'.runtime/verify_gen41_offline_tests_20261017.py').read_text())
    guard_calls=[n for n in ast.walk(guard_tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
        and n.func.attr=='write_text' and isinstance(n.func.value,ast.Name) and n.func.value.id=='bootstrap']
    assert len(guard_calls)==1;guard=ast.literal_eval(guard_calls[0].args[0])
    assert all(s in guard for s in ['socket.socket.connect = forbidden','socket.socket.connect_ex = forbidden',
        'socket.socket.sendto = forbidden','socket.create_connection = forbidden','socket.getaddrinfo = forbidden',
        "path.startswith('/dev/')",'sys.addaudithook(device_open)'])
    import_tree=ast.parse((W/'.runtime/verify_gen42_offline_tests_20261017.py').read_text())
    assignments=[n for n in import_tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='IMPORT_AUDIT' for t in n.targets)]
    assert len(assignments)==1;audit=ast.literal_eval(assignments[0].value)
    original="'remote_grasp6d' in name or name.startswith('alicia_grasp_modes')"
    assert audit.count(original)==1
    audit=audit.replace(original,"'remote_grasp6d' in name or Path(path).name in ('grasp_task_node.py', 'mode_aware_remote_grasp6d.py', 'unknown_tabletop_perception.py') or name.startswith('alicia_grasp_modes')")
    assert old_functions['bootstrap_sources']==new_functions['bootstrap_sources']
    old_plan_path=V/'gen53_full_parallel_r1/plan.json';assert sha(old_plan_path)==history['plan_sha256']
    old_plan=read(old_plan_path)
    assert old_plan['context']['guard_sha256']==hashlib.sha256(guard.encode()).hexdigest()
    assert old_plan['context']['audit_sha256']==hashlib.sha256(audit.encode()).hexdigest()
    guard_checks={
      'serial_and_guard_dependencies_sha_pinned':True,
      'python_socket_and_cpython_device_open_guard_byte_identical':True,
      'import_audit_script_identical_and_includes_task_remote_perception':True,
      'external_storage_mount_same_device_local_only_flags_retained':True,
      'plan_only_default_and_explicit_execute_retained':True,
      'resource_budget_rechecked_at_launch':True,
      'per_worker_external_tmp_ros_home_logs_bootstrap_and_xml_retained':True,
      'one_thread_blas_omp_and_plugin_autoload_disabled_retained':True,
      'full_mode_requires_closed_new_case_identities_and_full_scope_ready':True,
      'all_old_tests_byte_identical_required':True,
      'source_hash_and_generated_module_provenance_rechecks_retained':True,
      'exact_test_identity_union_no_missing_duplicate_or_unexpected_retained':True,
      'worker_evidence_and_bootstrap_readback_retained':True,
      'cleanup_only_coordinator_owned_offline_process_groups_retained':True,
    }
    for path,digest in pins.items():assert sha(path)==digest,'pinned input changed during review'
    result=dict(kind='GEN54_parallel_runner_independent_generation_and_baseline_review',passed=True,
        reviewed_runner=str(new_path),reviewed_runner_sha256=pins[new_path],preparation=str(V/'gen54_parallel_runner_preparation_r1/result.json'),
        preparation_sha256=sha(V/'gen54_parallel_runner_preparation_r1/result.json'),
        baseline_manifest=str(bpath),baseline_manifest_sha256=pins[bpath],baseline_source_count=1069,
        baseline_test_scope_files=322,baseline_unique_case_count=9010,baseline_all_source_bytes_exact=True,
        exact_simultaneous_approved_text_transform=True,normalized_AST_exact=True,
        function_count=len(new_functions),byte_identical_function_names=unchanged_functions,
        non_generation_or_verified_baseline_behavior_changes=False,
        guard_checks=guard_checks,guard_sha256=hashlib.sha256(guard.encode()).hexdigest(),
        import_audit_sha256=hashlib.sha256(audit.encode()).hexdigest(),
        historical_regression=dict(path=str(V/'gen53_full_parallel_r1/result.json'),xml_sha256=history['xml_sha256'],
            result_sha256=pins[V/'gen53_full_parallel_r1/result.json'],elapsed_sec=history['elapsed_sec'],workers=7,
            summary=summary,merged_XML_reconciles_exact_7_worker_case_union=True),
        current_review_test_bodies_executed=False,current_review_full_regression_performed=False,
        baseline_copy_is_not_a_new_test_pass=True,runner_main_or_worker_invoked=False,
        ROS_initialized=False,hardware_contacted=False,temperature_read=False,motion_commands_sent=False,
        production_source_modified=False,source_or_metadata_edited=False,
        limitation='Static rebind and archived-evidence review only; Python guard is not a native C SDK sandbox. Existing inert device fixtures remain required. No new candidate test run, geometry approval, installation or deployment is established.',
        references=[dict(path=str(p),sha256=digest) for p,digest in pins.items()],
        reviewer_helper=str(Path(__file__).resolve()),reviewer_helper_sha256=sha(__file__),
        elapsed_sec=time.monotonic()-started,cpu_sec=time.process_time()-cpu_started,created_ns=time.time_ns())
    O.mkdir()
    for name,value in [('result.json',result),('verified_source_files.json',checked),('verified_worker_results.json',worker_refs)]:
        with (O/name).open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    (O/'reviewed_diff.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(old_path),tofile=str(new_path))))
    (O/'summary.md').write_text('GEN54 并行离线 runner 独立复核通过。与 GEN53 runner 的全部差异精确等于代次、1069/322/9010 计数和已核验基线 SHA 的同时替换；没有其他代码行为变更。\n\n已逐文件验证 GEN54 baseline 的 1,069 份源码与 GEN53 发布/完整回归来源一致，322 个文件、9,010 个唯一用例与合并 XML 及 7 个 worker XML 的并集逐项一致。原隔离、导入来源、外盘 local_only、资源预留、用例无遗漏/无重复及仅清理所属离线子进程组的守卫保持。\n\n本次是静态代码与历史封口证据复核，不执行 runner 主程序或测试；复制基线不构成一次新的完整回归通过，也不批准候选几何修复、部署或实物运动。Python socket/CPython open 守卫不等同于 native SDK 沙箱，原有 inert device fixtures 约束继续有效。\n')
    print(json.dumps(dict(output=str(O),passed=True,result_sha256=sha(O/'result.json'),elapsed_sec=time.monotonic()-started),ensure_ascii=False))


if __name__=='__main__':main()
