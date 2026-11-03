"""Frozen-clock local diagnostic only; no ROS or execution authorization."""
import pathlib,json,sys,copy,time
sys.path[:0]=['.runtime/mujoco-3.2.3','tools']
import mujoco_digital_twin_server as server
out=pathlib.Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory'])
rows=json.load(open(out/'attempt1_simulation_summary.json'));results=[]
backend=server.MujocoDigitalTwinBackend()
for index in (0,4,9,10):
 src=pathlib.Path(rows[index]['path'])/'request.json';q=json.load(open(src));q['object_model']['mass_kg']=.02
 p=server._validate_v3_payload(q,now_sec=q['snapshot_stamp_sec']+.5,max_snapshot_age_sec=120.)
 started=time.monotonic();r=backend._simulate_validated_grasp(p,q['plan_id'])
 row=dict(source=str(src),scope='offline_frozen_time_no_execution_authority',changed_parameter='operator_declared_mass_upper_bound_kg',mass_kg=.02,response=r,elapsed_sec=time.monotonic()-started);results.append(row)
 print(index,r.get('simulation_ok'),r.get('score'),r.get('failure_code'),r.get('failure_reason'),flush=True)
 (out/'declared_mass_offline_replay.json').write_text(json.dumps(results,indent=2)+'\n')
