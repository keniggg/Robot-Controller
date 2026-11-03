"""Preserve bounded read-only source reconstruction scope, including failures."""
from pathlib import Path
import json,hashlib
W=Path(__file__).resolve().parents[1];B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');N=B/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1';O=N/'observation_pose_fix/offline_selection_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
a=json.loads((O/'source_attempt1/reconstruction.json').read_text());b=json.loads((O/'source_attempt2/reconstruction.json').read_text());s=json.loads((O/'source_attempt1/source.json').read_text())
files=[p for sub in ['source_attempt1','source_attempt2'] for p in (O/sub).iterdir() if p.is_file()]
source_names=['src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_view_source.py','src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_order.py','src/alicia_grasp_modes/src/alicia_grasp_modes/observation_arrival_projection.py','src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py']
r=dict(schema=1,status='source_reconstruction_incomplete_no_candidate_generation',source_stamp_ns=a['source_stamp_ns'],target_binding=s['source']['binding'],source_audit=dict(path=a['source_audit_path'],sha256=a['source_audit_sha256']),exact_checks=a['checks'],arrival_audit_exact=a['arrival_audit_exact'],support_point_count=a['support_point_count'],expected_support_point_count=a['expected_support_point_count'],tf1_vs_tf2_abs_delta=b['max_tf1_tf2_abs_delta'],tf2_checks=b['checks'],unresolved='Original source audit base optical transform bytes are not reproduced. Neither raw original T_base_optical nor original support array is retained in the inspected initial audit. TF1 and production TF2 BufferCore agree with each other; cause of audit hash mismatch remains unconfirmed. Equal point count is not proof of equal point coordinates.',candidate_generation_run=False,candidate_revision_used=None,strict_whole_arm_checked=False,complete_path_checked=False,execution_authority=False,planning_scene_connections=a['planning_scene_connections'],strict_replay_limitation='Closed bag has no moveit_msgs/PlanningScene, PlanningSceneWorld, or RobotState connection; original successful strict result covers its original endpoint only. New endpoint whole-arm collision and continuous path are unvalidated. No service replacement or mock success was used.',reusable_path=dict(module='alicia_grasp_modes.reachable_observation_views.generate_reachable_observation_views',fk='alicia_flexible_grasp.robot.observation_path_guard.SerialUrdfFk',model_path=s['model_path'],model_sha256=s['model_sha256'],maximum_seconds=s['original_search_audit']['max_seconds'],maximum_iterations=s['original_search_audit']['max_iterations'],numerical_threads=1),source_controls=dict(ROS_initialized=False,socket_attempts=[],temperature_values_read=False,image_extraction_once=True,second_bag_read_tf_only=True,bag_stat_unchanged=True,bag_full_rehashed=False,bag_sha256_from_closed_manifest=a['bag_sha256_from_closed_manifest']),artifacts=[dict(path=str(p),sha256=sha(p),size=p.stat().st_size) for p in sorted(files)],inspected_source=[dict(path=str(W/name),sha256=sha(W/name)) for name in source_names],report_helper_sha256=sha(Path(__file__)))
(O/'result.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
(O/'REPORT.md').write_text('''# GEN59 初始场景离线重建：范围与未完成项

本次停止在精确输入核验，尚未运行候选生成，也没有证明新姿态可达或整段路径安全。

来源为封口 GEN58 记录的初始观察规划源 `1792639754162577867`（request 2 / generation 3 / epoch 4 / g1-t4）。只提取一次精确深度和掩码；随后仅提取 TF 窄窗并保存原值。两次重建均未初始化 ROS、未建立 socket、未访问运动服务或预测服务。

已精确匹配原审计的 raw depth、raw mask、tool-camera 变换、保守目标八角点、完整 arrival projection 审计及相机参数；支撑点数量同为 105841。双侧缺口保留原始初始 surface 的八行，未用稍晚的重建几何替代。

未精确匹配 base-optical 变换、其派生 support point bytes 和 frozen tool reference。离线 TF1 与生产使用的 TF2 BufferCore 得到完全相同数值，仍与原审计哈希不同；原因尚未确认。没有调整、四舍五入或搜索变换数值来凑原哈希，也没有把相同点数当作点坐标一致。初始 audit 仅存变换哈希，已检查的摘要未保留原始完整矩阵；计划的新端点不能冒充初始相机变换。

可复用的纯离线路径是 `generate_reachable_observation_views` 加 `SerialUrdfFk`。已保存原 URDF SHA、测量关节、误差界、gripper 参数、原 2 秒 / 96 迭代 / 2 方向预算和 arrival 数组。取得精确原变换后可以继续候选端点验证；若另行授权使用重建 TF 作为几何代理，必须单列为近似探索。

即使端点生成通过，仍不能宣称严格运动可行：原袋没有 PlanningScene/PlanningSceneWorld/RobotState 连接；新姿态的全臂碰撞、连续轨迹和实际执行没有验证。原先那个观察姿态的 strict 成功不能转用给新姿态。

`result.json` 保存全部精确/失败 SHA、文件来源、代码 SHA 和可复用入口；`source_attempt1` 与 `source_attempt2` 保留两次有界重建结果，未覆盖第一次失败证据。
''')
(O/'summary_helper_source.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(dict(result_sha256=sha(O/'result.json'),report_sha256=sha(O/'REPORT.md'),status=r['status'])))
