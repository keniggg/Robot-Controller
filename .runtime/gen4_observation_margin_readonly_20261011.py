"""Sealed-bag projection comparison. No ROS subscription or service calls."""
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import rosbag
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix

from alicia_flexible_grasp.grasp.observation_working_volume import observation_obb_corners, _project
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

WORK = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_no_skin_20261011T000625Z'
REC = EXT / 'grasp_records/20261011T012545.114130Z_eeaa272415b840acbb66eb2c6bd76625'
AUDIT = REC / 'keyframes_audits/9e8c9caee267df85d8d5d9f308b85dc5d598777b0c1398906fce3181a7c4881f.json'
PLAN = '952c88e67ff5a28b3b74642a'
END_NS = 1791682023806032000


def matrix(transform):
    p = transform.transform.translation
    q = transform.transform.rotation
    result = quaternion_matrix([q.x, q.y, q.z, q.w])
    result[:3, 3] = [p.x, p.y, p.z]
    return result


data = json.loads(AUDIT.read_text())
selected = data['selected']
stage = selected['execution_sequence']['stages'][0]
buffer = tf2_ros.Buffer(rospy.Duration(500), debug=False)
objects, plans, statuses, configurations, feedback, goals = [], [], [], [], [], []
mount = None
topics = ['/tf', '/tf_static', '/grasp_record/camera_runtime_metadata',
          '/grasp_6d/preview_plan_enriched', '/perception/object',
          '/perception/unknown/detector_status', '/alicia_d/accepted_joint_states',
          '/alicia_controller/follow_joint_trajectory/goal']
with rosbag.Bag(str(REC / 'keyframes.bag')) as bag:
    for topic, msg, received in bag.read_messages(topics=topics):
        ns = received.to_nsec()
        if topic in ('/tf', '/tf_static'):
            for transform in msg.transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(transform, 'sealed-bag')
                else:
                    buffer.set_transform(transform, 'sealed-bag')
                if transform.header.frame_id == 'tool0' and transform.child_frame_id == 'camera_link':
                    mount = matrix(transform)
        elif topic == '/grasp_record/camera_runtime_metadata':
            configurations.append((ns, json.loads(msg.data)['parameters']))
        elif topic == '/grasp_6d/preview_plan_enriched' and msg.plan_id == PLAN:
            plans.append(msg)
        elif topic == '/perception/object' and 1791682021500000000 < ns < END_NS:
            objects.append((ns, msg))
        elif topic == '/perception/unknown/detector_status' and 1791682021500000000 < ns < END_NS:
            statuses.append((ns, json.loads(msg.data)))
        elif topic == '/alicia_d/accepted_joint_states' and 1791682020000000000 < ns < END_NS:
            feedback.append((ns, msg.header.stamp.to_nsec(), list(msg.name), list(msg.position)))
        elif topic == '/alicia_controller/follow_joint_trajectory/goal' and ns > 1791682010000000000:
            goals.append((ns, msg))

configuration = [row for row in configurations if row[0] < END_NS][-1][1]
intrinsics = [configuration[key] for key in ('fx', 'fy', 'cx', 'cy')]
width, height = configuration['width'], configuration['height']
rgb_valid = rectification_rgb_valid_mask(configuration, (height, width))
if rgb_valid is None:
    rgb_valid = np.ones((height, width), dtype=bool)
plan = plans[-1]
geometry = plan.object_geometry
position, quaternion = geometry.pose_base.position, geometry.pose_base.orientation
rotation = quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[:3, :3]
center = np.array([position.x, position.y, position.z])
size = np.array([geometry.size_xyz_m.x, geometry.size_xyz_m.y, geometry.size_xyz_m.z])
corners = observation_obb_corners(center, rotation, size)
prism_source = selected['observation_view']['arrival_projection_endpoint']['source']
prism_reference = json.loads(prism_source['reference_binding'])
footprint = np.array(prism_reference['reference_support_footprint_base'])
normal = np.array(prism_reference['reference_support_plane_base'][:3])
normal /= np.linalg.norm(normal)
prism = np.vstack((footprint, footprint + normal * prism_source['measured_height_m']))


def projection(points, camera):
    uv, depth = _project(points, camera, intrinsics)
    low, high = uv.min(axis=0), uv.max(axis=0)
    edge = float(np.min(np.column_stack((uv[:, 0], width - 1 - uv[:, 0],
                                         uv[:, 1], height - 1 - uv[:, 1]))))
    box = [int(np.floor(low[0])) - 16, int(np.floor(low[1])) - 16,
           int(np.ceil(high[0])) + 17, int(np.ceil(high[1])) + 17]
    x0, y0, x1, y1 = box
    in_canvas = 0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height
    return dict(uv_min_px=low.tolist(), uv_max_px=high.tolist(),
                minimum_image_edge_distance_px=edge, required_margin_px=36.,
                passes_36px=edge >= 36., minimum_depth_m=float(depth.min()),
                maximum_depth_m=float(depth.max()), background16_box_xyxy=box,
                background16_inside_canvas=in_canvas,
                background16_invalid_rgb_pixels=(int(np.count_nonzero(~rgb_valid[y0:y1, x0:x1]))
                                                  if in_canvas else None))


planned_tool = quaternion_matrix(stage['quaternion_xyzw'])
planned_tool[:3, 3] = stage['position_m']
planned_camera = planned_tool @ mount
goal_q = np.array(json.loads(re.search(r'observation_goal_joint_positions=(\[[^]]+\])',
                                     selected['moveit']['reason']).group(1)))
model_path = WORK / 'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
names = ['Joint%d' % i for i in range(1, 7)]
fk = SerialUrdfFk(model_path.read_text(), names)
strict_camera = fk(goal_q) @ mount
rows = []
for received_ns, obj in objects:
    source = obj.header.stamp
    camera = matrix(buffer.lookup_transform('base_link', 'camera_link', source))
    tool = matrix(buffer.lookup_transform('base_link', 'tool0', source))
    cosine = (np.trace(planned_tool[:3, :3].T @ tool[:3, :3]) - 1.) * .5
    before = [row for row in feedback if row[1] <= source.to_nsec()]
    q = None
    if before:
        last = max(before, key=lambda row: row[1])
        q = np.array([last[3][last[2].index(name)] for name in names])
    rows.append(dict(received_ns=received_ns, source_ns=source.to_nsec(), detected=obj.detected,
                     obb_projection=projection(corners, camera), prism_projection=projection(prism, camera),
                     base_from_camera=camera.tolist(),
                     tool_position_error_mm=float(np.linalg.norm(planned_tool[:3, 3] - tool[:3, 3]) * 1000),
                     tool_orientation_error_deg=float(np.degrees(np.arccos(np.clip(cosine, -1., 1.)))),
                     accepted_q_before_source=None if q is None else q.tolist(),
                     strict_goal_minus_actual_rad=None if q is None else (goal_q - q).tolist()))

report = dict(scope='offline_sealed_GEN4_bag_projection_no_ROS_API_or_motion',
              record=str(REC), audit=str(AUDIT), audit_sha256=hashlib.sha256(AUDIT.read_bytes()).hexdigest(),
              plan_id=PLAN, frozen_geometry_source_ns=geometry.header.stamp.to_nsec(),
              geometry_center_base_m=center.tolist(), geometry_size_xyz_m=size.tolist(),
              geometry_axes_base=rotation.tolist(),
              camera=dict(width=width, height=height, intrinsics=intrinsics,
                          intrinsics_source=configuration['intrinsics_source'],
                          rgb_valid_pixels=int(np.count_nonzero(rgb_valid)),
                          rgb_canvas_pixels=width*height),
              mount_tool_from_camera=mount.tolist(), model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
              planned=dict(obb=projection(corners, planned_camera), prism=projection(prism, planned_camera),
                           base_from_camera=planned_camera.tolist(),
                           logged_obb=selected['observation_view']['working_volume']),
              strict_command_goal=dict(q=goal_q.tolist(), obb=projection(corners, strict_camera),
                                       prism=projection(prism, strict_camera)),
              measured_source_frames=rows,
              actual_wire_goals=[dict(received_ns=ns, names=list(msg.goal.trajectory.joint_names),
                                     goal_positions=list(msg.goal.trajectory.points[-1].positions),
                                     duration_sec=msg.goal.trajectory.points[-1].time_from_start.to_sec())
                                 for ns, msg in goals],
              statuses=[dict(received_ns=ns, source_stamp_ns=d.get('source_stamp_ns'),
                             state=d.get('state'), detail=d.get('detail')) for ns, d in statuses],
              limitation='Recorder order approximates task callback delivery. All saved candidate target messages in the acceptance window are checked; exact task-selected object timestamp was not logged.')
if not Path('/media/zhuyupei/SanStand').is_mount():
    raise SystemExit('SanStand not mounted')
out = NEW / 'gen4_analysis'
out.mkdir(parents=True, exist_ok=True)
target = out / 'planned_vs_measured_observation_projection.json'
target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({'report': str(target), 'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                  'planned': report['planned']['obb'], 'strict': report['strict_command_goal']['obb'],
                  'measured': [{k:v for k,v in row.items() if k!='base_from_camera'} for row in rows]},
                 ensure_ascii=False))
