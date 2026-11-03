"""Read-only same-frame depth/context comparison at stationary hardware."""
import json,pathlib,time
from types import SimpleNamespace as NS
import rospy,tf2_ros,numpy as np,cv2
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp_supervisor.msg import ObjectPose
from alicia_flexible_grasp.vision.object_geometry import _expanded_bbox_mask,deproject_depth,_fit_support_plane
p=pathlib.Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory']);rospy.init_node('support_context_same_frame_readonly',anonymous=True,disable_signals=True)
buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf);bridge=CvBridge();time.sleep(.8)
obj=rospy.wait_for_message('/perception/object',ObjectPose,timeout=4);mask=rospy.wait_for_message('/perception/object_mask',Image,timeout=4);depth=rospy.wait_for_message('/supervisor/camera/depth/image_raw',Image,timeout=4);state=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=4)
a=json.load(open(p/'current_observation_direct_audit.json'))
tr=buf.lookup_transform('base_link',depth.header.frame_id,depth.header.stamp,rospy.Duration(2)).transform;T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z];O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T=T@O
D=bridge.imgmsg_to_cv2(depth,'passthrough');M=bridge.imgmsg_to_cv2(mask,'mono8');np.savez_compressed(p/'live_support_same_frame_inputs.npz',depth=D,mask=M,T_base_optical=T)
c=rospy.get_param('/camera');K=NS(width=D.shape[1],height=D.shape[0],**{k:c[k] for k in ('fx','fy','cx','cy')});bbox=a['graspnet_input']['detected_bbox_xywh'];out=dict(scope='diagnostic_only_stored_ROI_current_depth_no_execution_authority',depth_stamp_ns=depth.header.stamp.to_nsec(),mask_stamp_ns=mask.header.stamp.to_nsec(),object_stamp_ns=obj.header.stamp.to_nsec(),bbox=bbox,camera=c,T_base_optical=T.tolist(),joint_names=list(state.name),joint_positions=list(state.position),fits=[])
for ratio in (.3,.6,1.,1.5,2.5):
 for exclusion in ('mask','bbox'):
  expanded,bm=_expanded_bbox_mask(D.shape,bbox,ratio);sm=expanded & ~(M>0 if exclusion=='mask' else bm);P,_=deproject_depth(D,sm,K,c['depth_scale']);P=P@T[:3,:3].T+T[:3,3];n,d,frac=_fit_support_plane(P,.0035,200)
  if n[2]<0:n=-n;d=-d
  err=P@n+d;inlier=abs(err)<=.0035;item=dict(expand_ratio=ratio,exclusion=exclusion,normal=n.tolist(),offset_m=d,inlier_ratio=frac,inlier_rms_m=float(np.sqrt(np.mean(err[inlier]**2))))
  out['fits'].append(item);print(ratio,exclusion,np.round(n,4),round(d,5),round(frac,3),flush=True)
(p/'live_support_same_frame_comparison.json').write_text(json.dumps(out,indent=2)+'\n')
