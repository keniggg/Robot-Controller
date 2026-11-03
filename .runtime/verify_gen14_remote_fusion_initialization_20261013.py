#!/usr/bin/env python3
"""No ROS master or robot: validate the original historical fused surface."""
import argparse,json,hashlib,sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch,Mock
import rospy
from cv_bridge import CvBridge
from gen14_remote_fusion_fixture_20261013 import W,N,F,load_recorded_parameters,digest
from gen14_three_owner_remote_20261013 import build_remote

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--candidate',required=True);args=p.parse_args()
output=Path(args.output);assert F in output.parents and not output.exists();output.mkdir()
fixture=json.loads((F/'fixture.json').read_text())
executing=next(row['message'] for row in fixture['wire'] if row['topic'].endswith('measurement_request') and row['message']['state']=='executing')
params,parameter_audit=load_recorded_parameters(fixture['camera'],fixture['unknown_parameters'],executing['selection'])
dirs=[F/'initial_surface_preceding'/str(ns) for ns in (1791860167282787322,1791860167481009244)]+[F/'initial_surface_fixture']
now=executing['stamp_ns'];clock=lambda:now
def unreachable(*args,**kwargs):raise AssertionError('No arrival-time TF request is allowed during historical initialization')
with patch.object(rospy,'get_param',lambda name,default=None:params.get(name,default)),patch.object(rospy,'get_param_cached',lambda name,default=None:params.get(name,default)),patch.object(rospy,'get_master',lambda:SimpleNamespace(getParam=lambda name:(1,'saved',params.get(name)))),patch.object(rospy.Time,'now',staticmethod(lambda:rospy.Time(now//10**9,now%10**9))),patch.object(rospy,'loginfo',Mock()),patch.object(rospy,'logwarn',Mock()),patch.object(rospy,'logwarn_throttle',Mock()):
    node,module,seed,audit=build_remote(args.candidate,SimpleNamespace(bridge=CvBridge()),clock,executing,F/'three_owner_seed',parameters=params,lookup_transform=unreachable,initial_surface_directories=dirs)
result=dict(passed=True,scope='Historical initialization only; no arrival qualification or phase processing claimed',parameter_audit=parameter_audit,remote=audit,helper_sha256={str(W/'.runtime'/name):digest(W/'.runtime'/name) for name in ('gen14_remote_fusion_fixture_20261013.py','gen14_three_owner_remote_20261013.py','verify_gen14_remote_fusion_initialization_20261013.py')},no_ros_master=True,motion_commands_sent=False)
(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(audit['initial_surface_reconstruction'],indent=2))
