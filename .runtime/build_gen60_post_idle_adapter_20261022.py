"""Offline authoring helper: clone preserved GEN59 sampler with GEN60 bindings."""
from pathlib import Path
W=Path(__file__).resolve().parents[1]
old=W/'.runtime/read_gen59_post_attempt_20261022.py'
raw=old.read_text()
def replace(before,after):
 global raw
 assert raw.count(before)==1,before[:100]
 raw=raw.replace(before,after)
replace('Read GEN59 terminal/manual SDK state.', 'Read GEN60 mixed-runtime terminal/manual SDK state.')
replace('import hashlib\n','import hashlib\nimport importlib.util\nimport sys\n')
replace("RELEASE = N / 'verification/gen59_release_manifest.json'\nACCEPTED = O / 'runtime_reload_r1/acceptance/result.json'\nPINS = {\n    RELEASE: '5fed4cbed7ea1e824eda3bb750657970dcc1ec2d711cd041eff1a3d482ef2c6c',\n    ACCEPTED: 'b5b53dcf13f51d507187a4c71e7ac20977e3bbc1ed6b51025f30ed5aabfb1082',\n    W / '.runtime/read_gen58_post_attempt_20261021.py': '5c3b5bc908838506637684771c051383b0e06405921b9725ff8beea4751ced8b',\n}","""E60 = O / 'held_lattice_fix_r1'
RELEASE = N / 'verification/gen60_release_manifest.json'
ACCEPTED = E60 / 'runtime_reload_r1/acceptance/result.json'
RUNTIME = W / '.runtime/gen60_gateway_reload_common_20261022.py'
PINS = {W / '.runtime/read_gen59_post_attempt_20261022.py':
    '51a588cfc1ea13f5c9fba5e33ed3373503f1dd3b26011cebc5780abcfeee0a69'}
sys.dont_write_bytecode = True""")
replace('def verify_sources(source):','def verify_sources(source):')
pos=raw.index('\ndef identities(nodes):')
raw=raw[:pos]+'''

def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def read(path):
    return json.loads(Path(path).read_bytes())


def selection_identity(value):
    from alicia_grasp_modes.selection import parse_selection
    require(type(value) is dict and set(value)=={'mode','strategy','generation','stamp_ns'},
            'complete exact selection required')
    return parse_selection(value)


def returned_selection(entry, runner_exit, authorization, deployment, expected_generation):
    require(type(expected_generation) is int and expected_generation==4,
            'this one GEN60 attempt expects generation4')
    require(entry['kind']=='GEN60_remote_one_recorded_attempt_recorded'
        and entry['original_runner_returned'] is True and entry['gate_returned_to_original_mode_line'] is True
        and entry['automatic_retry'] is False and entry['new_motion_authority_created'] is False,
        'current original runner did not return after its guarded boundary')
    require(type(entry['runner_returncode']) is int and entry['runner_returncode']==runner_exit['runner_returncode']
        and runner_exit['physical_success']=='unverified','runner return evidence differs')
    selected=selection_identity(runner_exit['selection'])
    prior=selection_identity(deployment['selection'])
    require(selected==selection_identity(authorization['selection'])
        and prior==selection_identity(authorization['previous_selection'])
        and selected['generation']==expected_generation==prior['generation']+1
        and selected['mode']==prior['mode']=='unknown'
        and selected['strategy']==prior['strategy']=='observe_return'
        and selected['stamp_ns']>prior['stamp_ns'], 'exact returned selection/predecessor differs')
    require(authorization['selection_response']['success'] is True
        and authorization['selection_commit']['latched_selection_matched'] is True,
        'selection commit was not acknowledged')
    return selected


def load_bindings(args):
    require(args.release.resolve(strict=True)==RELEASE and args.deployment.resolve(strict=True)==ACCEPTED,
            'unexpected GEN60 release/acceptance')
    supplied={RELEASE:args.release_sha256,ACCEPTED:args.deployment_sha256,RUNTIME:args.runtime_common_sha256,
              Path(__file__).resolve():args.helper_sha256,args.attempt_result.resolve(strict=True):args.attempt_result_sha256,
              args.runner_exit.resolve(strict=True):args.runner_exit_sha256}
    PINS.update(supplied)
    require(all(sha(p)==h for p,h in PINS.items()),'post-idle evidence/helper changed')
    deployment=read(ACCEPTED);release=read(RELEASE)
    require(deployment['source_manifest']==str(RELEASE) and deployment['source_manifest_sha256']==args.release_sha256
        and deployment['input_sha256'][str(RUNTIME)]==args.runtime_common_sha256,'accepted runtime/source differs')
    spec=importlib.util.spec_from_file_location('gen60_post_idle_runtime',RUNTIME)
    runtime=importlib.util.module_from_spec(spec);spec.loader.exec_module(runtime)
    ctx=runtime.accepted_context(ACCEPTED,args.deployment_sha256)
    require(ctx['acceptance']==deployment and ctx['release']==release,'validated GEN60 acceptance differs')
    for p,h in ctx['pins'].items():PINS[Path(p)]=h
    source=release['source_sha256'];require(len(source)==1097,'source closure differs')
    entry_path=args.attempt_result.resolve();exit_path=args.runner_exit.resolve()
    entry=read(entry_path);runner_exit=read(exit_path)
    require(entry_path.parent.parent==E60/'actual_attempt_r1' and entry_path.parent.name.startswith('scene_start_')
        and entry_path.name=='recorded_entry_result.json','unexpected current attempt entry path')
    record=Path(entry['record_dir']).resolve(strict=True)
    require(record.parent==Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records')
        and exit_path==record/'user_request_runner_exit.json','runner exit not bound to current record')
    require(entry['input_sha256'][str(RELEASE)]==args.release_sha256
        and entry['input_sha256'][str(ACCEPTED)]==args.deployment_sha256,'returned attempt was another deployment')
    auth_path=record/'authorization.json';authorization=read(auth_path);PINS[auth_path]=sha(auth_path)
    require(Path(authorization['session_dir']).resolve()==entry_path.parent,'record authorization scene differs')
    selected=returned_selection(entry,runner_exit,authorization,deployment,args.expected_generation)
    verify_sources(source)
    return runtime,ctx,release,deployment,selected,record

''' + raw[pos:]
replace("    args = ap.parse_args()\n    assert args.expected_generation > 0", """    for key in ('release','deployment','attempt-result','runner-exit'):
        ap.add_argument('--'+key,type=Path,required=True)
        ap.add_argument('--'+key+'-sha256',required=True)
    ap.add_argument('--runtime-common-sha256',required=True)
    ap.add_argument('--helper-sha256',required=True)
    args = ap.parse_args()
    assert args.expected_generation == 4""")
replace("assert (O / 'actual_attempt_r1').resolve() in out.parents", "assert (E60 / 'actual_attempt_r1').resolve() in out.parents")
start=raw.index('        for path, expected in PINS.items():\n',raw.index('def main():'))
end=raw.index('        import rospy\n',start)
raw=raw[:start]+'''        runtime,ctx,release,deployment,selected,record=load_bindings(args)
        source=release['source_sha256'];nodes=deployment['full_runtime_nodes']
        require(len(nodes)==17 and nodes==deployment['final_all_registered_nodes'], 'complete GEN60 inventory differs')
        before=identities(nodes)
        result.update(input_sha256={str(p):h for p,h in PINS.items()},record_dir=str(record),
            source_manifest=str(RELEASE),source_manifest_sha256=args.release_sha256,
            deployment=str(ACCEPTED),deployment_sha256=args.deployment_sha256,
            actual_returned_selection=selected,composite_runtime=deployment['composite_runtime'])
''' +raw[end:]
replace("rospy.init_node('gen59_post_attempt_readonly', anonymous=True, disable_rosout=True)\n        selection_before = parse_selection(rospy.get_param('/grasp_mode/selection'))", """rospy.init_node('gen60_post_attempt_readonly', anonymous=True, disable_rosout=True)
        import rosgraph
        master=rosgraph.Master(rospy.get_name())
        graph_before=runtime.graph(ctx,rospy,master,gateway=nodes[runtime.GATEWAY])
        selection_before = selection_identity(rospy.get_param('/grasp_mode/selection'))
        require(selection_before==selected,'live selection is not exact returned attempt selection')""")
replace("('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray, None)","('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray, '/bessica_d_hw_interface')")
replace("('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray, None)","('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray, '/bessica_d_hw_interface')")
replace("selection = parse_selection(rospy.get_param('/grasp_mode/selection'))", "selection = selection_identity(rospy.get_param('/grasp_mode/selection'))")
replace("        after = identities(nodes)\n        verify_sources(source)", """        after = identities(nodes)
        graph_after=runtime.graph(ctx,rospy,master,gateway=nodes[runtime.GATEWAY],auxiliaries=graph_before['auxiliary_nodes'])
        require(graph_before==graph_after,'instance graph or publishers changed during post-idle sampling')
        for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
            require(graph_after['publishers'].get(topic)==['/bessica_d_hw_interface'],'action status owner differs')
        verify_sources(source)
        runtime.recheck(ctx,source)""")
replace("accepted_processes_unchanged=True, source_count=1094,", "accepted_processes_unchanged=True, source_count=1097,\n                      ownership_before=graph_before,ownership_after=graph_after,")
new=W/'.runtime/read_gen60_post_attempt_20261022.py'
assert not new.exists()
new.write_text(raw)
