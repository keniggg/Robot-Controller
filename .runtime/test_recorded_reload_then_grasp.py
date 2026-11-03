"""The real loader crosses ROS XMLRPC string/int source stamps; never compare raw dicts."""
import importlib.util,json
from pathlib import Path
import pytest
p=Path(__file__).with_name('recorded_reload_then_grasp_20261004.py')
spec=importlib.util.spec_from_file_location('recorded_reload',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def records():
    selected=dict(mode='unknown',strategy='observe_return',generation=10,stamp_ns='1791100539470502138')
    report=dict(loaded=True,preserved_nodes_unchanged=True,prestart_retry=dict(returncode=0,selection=selected))
    event=dict(new_selection=dict(selected,stamp_ns=int(selected['stamp_ns'])),
        preview_only=False,failure=None,path=str(m.EXT/'ros_log/new_selection'))
    return report,event

def test_exact_xmlrpc_string_stamp_matches_ros_json_integer_without_rewriting_receipt():
    report,event=records()
    assert m.exact_selection_result(report,json.dumps(event))==m.EXT/'ros_log/new_selection/result.json'
    assert isinstance(report['prestart_retry']['selection']['stamp_ns'],str)
    assert isinstance(event['new_selection']['stamp_ns'],int)

@pytest.mark.parametrize('change',['generation','stamp','preview','failure','missing','duplicate','path','returncode','load'])
def test_wrong_or_unverified_handoff_never_launches_grasp(change):
    report,event=records();suffix=''
    if change=='generation':event['new_selection']['generation']+=1
    elif change=='stamp':event['new_selection']['stamp_ns']+=1
    elif change=='preview':event['preview_only']=True
    elif change=='failure':event['failure']='rejected'
    elif change=='missing':event={}
    elif change=='duplicate':suffix='\n'+json.dumps(event)
    elif change=='path':event['path']='/tmp/unverified'
    elif change=='returncode':report['prestart_retry']['returncode']=1
    elif change=='load':report['loaded']=False
    with pytest.raises(RuntimeError):m.exact_selection_result(report,json.dumps(event)+suffix)
