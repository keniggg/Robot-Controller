#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[2]
TRAJECTORY_EXECUTION = ROOT / 'real-arm' / 'alicia_d_moveit' / 'launch' / 'trajectory_execution.launch.xml'
CONTROLLERS = ROOT / 'real-arm' / 'alicia_d_driver' / 'config' / 'controllers.yaml'
GRASP_PARAMS = ROOT / 'alicia_flexible_grasp_supervisor' / 'config' / 'grasp_params.yaml'
DRIVER_LAUNCHES = (
    ROOT / 'real-arm' / 'alicia_d_driver' / 'launch' / 'alicia_d_driver.launch',
    ROOT / 'real-arm' / 'alicia_d_driver' / 'launch' / 'alicia_d_bringup.launch',
)
DRIVER_SOURCE = (
    ROOT / 'real-arm' / 'alicia_d_driver' / 'src' / 'alicia_d_driver_node.cpp'
)
DRIVER_HEADER = (
    ROOT / 'real-arm' / 'alicia_d_driver' / 'include'
    / 'alicia_d_driver' / 'alicia_d_driver_node.hpp'
)
ENDPOINT_TRIM_ADMISSION = (
    ROOT / 'real-arm' / 'alicia_d_driver' / 'include'
    / 'alicia_d_driver' / 'endpoint_trim_driver_admission.hpp'
)


class MoveItTrajectoryExecutionConfigTest(unittest.TestCase):
    def test_allowed_start_tolerance_accepts_real_arm_feedback_lag(self):
        tree = ET.parse(str(TRAJECTORY_EXECUTION))
        values = [
            node.attrib.get('value')
            for node in tree.findall('.//param')
            if node.attrib.get('name') == 'trajectory_execution/allowed_start_tolerance'
        ]

        self.assertTrue(values)
        self.assertGreaterEqual(float(values[0]), 0.05)

    def test_goal_duration_margin_matches_slow_hardware_goal_window(self):
        tree = ET.parse(str(TRAJECTORY_EXECUTION))
        values = [
            node.attrib.get('value')
            for node in tree.findall('.//param')
            if node.attrib.get('name') == 'trajectory_execution/allowed_goal_duration_margin'
        ]

        self.assertTrue(values)
        self.assertGreaterEqual(float(values[0]), 8.0)

    def test_real_arm_goal_tolerance_allows_measured_pregrasp_settle_error(self):
        data = yaml.safe_load(CONTROLLERS.read_text())
        constraints = data['alicia_controller']['constraints']

        for joint_name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
            with self.subTest(joint_name=joint_name):
                self.assertGreaterEqual(float(constraints[joint_name]['goal']), 0.03)

    def test_real_arm_goal_tolerance_keeps_quantized_boundary_from_aborting_pregrasp(self):
        controller_data = yaml.safe_load(CONTROLLERS.read_text())
        constraints = controller_data['alicia_controller']['constraints']
        grasp_data = yaml.safe_load(GRASP_PARAMS.read_text())
        robot_cfg = grasp_data['robot']
        effective_supervisor_tolerance = (
            float(robot_cfg['execution_goal_tolerance_rad'])
            + float(robot_cfg['execution_goal_tolerance_slack_rad'])
        )

        self.assertGreaterEqual(effective_supervisor_tolerance + 1e-12, 0.035)
        for joint_name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
            with self.subTest(joint_name=joint_name):
                self.assertGreaterEqual(float(constraints[joint_name]['goal']), 0.035)

    def test_strict_pregrasp_execution_is_retimed_for_real_hardware(self):
        grasp_data = yaml.safe_load(GRASP_PARAMS.read_text())
        robot_cfg = grasp_data['robot']

        self.assertIs(robot_cfg['strict_execution_retime_enabled'], True)
        self.assertLessEqual(
            float(robot_cfg['strict_execution_max_joint_velocity_rad_s']),
            0.08,
        )
        joint_limits = robot_cfg[
            'strict_execution_joint_velocity_limits_rad_s'
        ]
        self.assertLessEqual(float(joint_limits['Joint3']), 0.02)
        self.assertGreater(float(joint_limits['Joint3']), 0.0)
        self.assertLessEqual(float(joint_limits['Joint4']), 0.02)
        self.assertGreater(float(joint_limits['Joint4']), 0.0)
        self.assertLessEqual(float(joint_limits['Joint6']), 0.02)
        self.assertGreater(float(joint_limits['Joint6']), 0.0)
        self.assertGreaterEqual(
            float(robot_cfg['strict_execution_min_duration_sec']),
            3.0,
        )
        self.assertGreater(
            float(robot_cfg['strict_execution_velocity_scaling']),
            0.0,
        )
        self.assertLess(
            float(robot_cfg['strict_execution_start_tolerance_rad']),
            0.12,
        )

    def test_controller_sync_accepts_a_previously_successful_endpoint(self):
        grasp_data = yaml.safe_load(GRASP_PARAMS.read_text())
        robot_cfg = grasp_data['robot']
        endpoint_tolerance = (
            float(robot_cfg['execution_goal_tolerance_rad'])
            + float(robot_cfg['execution_goal_tolerance_slack_rad'])
        )
        sync_tolerance = float(
            robot_cfg['strict_execution_controller_sync_tolerance_rad']
        )

        self.assertGreaterEqual(
            sync_tolerance + 1e-12,
            endpoint_tolerance,
        )
        self.assertLess(
            sync_tolerance,
            float(robot_cfg['strict_execution_start_tolerance_rad']),
        )

    def test_driver_reasserts_held_sdk_target_at_feedback_cadence(self):
        for launch_path in DRIVER_LAUNCHES:
            with self.subTest(launch_path=launch_path.name):
                tree = ET.parse(str(launch_path))
                params = {
                    node.attrib.get('name'): node.attrib.get('value')
                    for node in tree.findall('.//param')
                }
                keepalive_hz = float(params['command_keepalive_rate_hz'])
                feedback_hz = float(params['state_poll_rate_hz'])
                command_hz = float(params['command_rate_hz'])

                self.assertGreater(keepalive_hz, 0.0)
                self.assertGreaterEqual(keepalive_hz, feedback_hz)
                self.assertLessEqual(keepalive_hz, command_hz)

    def test_driver_defers_diagnostic_queries_until_motion_is_measured(self):
        source = DRIVER_SOURCE.read_text()
        header = DRIVER_HEADER.read_text()

        self.assertIn(
            'diagnostic_query_suppressed_for_motion(now)',
            source,
        )
        self.assertIn(
            'const bool temperature_due = !diagnostic_queries_suppressed',
            source,
        )
        self.assertIn(
            'const bool self_check_due = !diagnostic_queries_suppressed',
            source,
        )
        self.assertIn(
            'std::abs(target - feedback) >',
            source,
        )
        self.assertIn(
            'std::abs(target_gripper_rad - feedback_gripper_rad) >',
            source,
        )
        self.assertIn(
            'last_motion_reference_change_time_ = command_time;',
            source,
        )
        self.assertIn(
            'bool suppress_diagnostic_queries_while_motion_active_ = true;',
            header,
        )

        for launch_path in DRIVER_LAUNCHES:
            with self.subTest(launch_path=launch_path.name):
                tree = ET.parse(str(launch_path))
                params = {
                    node.attrib.get('name'): node.attrib.get('value')
                    for node in tree.findall('.//param')
                }
                self.assertEqual(
                    params['suppress_diagnostic_queries_while_motion_active'],
                    'true',
                )
                self.assertGreater(
                    float(params['diagnostic_query_motion_quiet_sec']),
                    0.0,
                )
                self.assertGreater(
                    float(params['diagnostic_query_motion_error_rad']),
                    0.0,
                )

    def test_driver_direct_start_default_keeps_final_target_alive(self):
        source = DRIVER_SOURCE.read_text()

        self.assertIn(
            'pnh_.param<double>("command_keepalive_rate_hz", '
            'command_keepalive_rate_hz_, 10.0);',
            source,
        )
        self.assertIn(
            'std::min(command_keepalive_rate_hz_, command_rate_hz_)',
            source,
        )

    def test_driver_first_partial_arm_command_inherits_real_gripper_feedback(self):
        source = DRIVER_SOURCE.read_text()
        seed_block = source.split(
            'if (should_seed_command_state) {',
            1,
        )[1].split('}', 1)[0]

        self.assertIn(
            'latest_joint_angles_ = current_joint_positions_;',
            seed_block,
        )
        self.assertIn(
            'latest_gripper_rad_ = current_gripper_position_;',
            seed_block,
        )
        self.assertNotIn(
            'has_latest_command_ = true',
            seed_block,
        )

    def test_driver_endpoint_feedback_trim_is_off_in_ordinary_control_path(self):
        controller_data = yaml.safe_load(CONTROLLERS.read_text())
        constraints = controller_data['alicia_controller']['constraints']
        goal_bound = min(
            float(constraints[name]['goal'])
            for name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        )
        path_bound = min(
            float(constraints[name]['trajectory'])
            for name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        )
        grasp_data = yaml.safe_load(GRASP_PARAMS.read_text())
        controller_settle_sec = float(
            grasp_data['robot']['strict_execution_controller_sync_duration_sec']
        )

        for launch_path in DRIVER_LAUNCHES:
            with self.subTest(launch_path=launch_path.name):
                tree = ET.parse(str(launch_path))
                params = {
                    node.attrib.get('name'): node.attrib.get('value')
                    for node in tree.findall('.//param')
                }

                self.assertEqual(
                    params['endpoint_feedback_trim_enabled'],
                    'false',
                )
                self.assertGreaterEqual(
                    float(params['endpoint_feedback_trim_stable_sec']),
                    controller_settle_sec,
                )
                self.assertAlmostEqual(
                    float(params['endpoint_feedback_trim_activation_error_rad']),
                    goal_bound,
                )
                self.assertGreater(
                    float(params['endpoint_feedback_trim_max_rad']),
                    float(params['endpoint_feedback_trim_activation_error_rad']),
                )
                self.assertLessEqual(
                    float(params['endpoint_feedback_trim_max_rad']),
                    path_bound,
                )
                self.assertGreater(
                    float(params['endpoint_feedback_trim_gain']),
                    0.0,
                )
                self.assertLessEqual(
                    float(params['endpoint_feedback_trim_gain']),
                    1.0,
                )

        source = DRIVER_SOURCE.read_text()
        header = DRIVER_HEADER.read_text()
        self.assertIn(
            'pnh_.param<bool>("endpoint_feedback_trim_enabled", '
            'endpoint_feedback_trim_enabled_, false);',
            source,
        )
        self.assertIn(
            'bool endpoint_feedback_trim_enabled_ = false;',
            header,
        )

        grasp_data = yaml.safe_load(GRASP_PARAMS.read_text())
        self.assertEqual(
            int(grasp_data['grasp']['observation_endpoint_correction_attempts']),
            0,
        )

    def test_driver_endpoint_feedback_trim_is_live_and_joint_agnostic(self):
        source = DRIVER_SOURCE.read_text()
        header = DRIVER_HEADER.read_text()
        timer = source.split(
            'void AliciaDDriverNode::send_command_timer_callback', 1
        )[1].split('void AliciaDDriverNode::process_serial_data_callback', 1)[0]

        self.assertIn(
            '#include "alicia_d_driver/endpoint_trim_continuity.hpp"',
            header,
        )
        self.assertIn(
            '#include "alicia_d_driver/endpoint_trim_driver_admission.hpp"',
            header,
        )
        self.assertIn('EndpointTrimContinuity endpoint_trim_continuity_;', header)
        self.assertIn('EndpointTrimCommandOrder endpoint_trim_command_order_;', header)
        self.assertIn(
            'endpoint_trim_config_.response_min_rad =',
            source,
        )
        self.assertIn(
            'endpoint_feedback_trim_response_min_quantums_ *',
            source,
        )
        self.assertIn('endpoint_trim_continuity_.note_feedback(', timer)
        self.assertIn('endpoint_trim_continuity_.update(now.toSec())', timer)
        self.assertIn('endpoint_trim_continuity_.request_correction(', timer)
        self.assertIn('endpoint_trim_stream_target(', timer)
        self.assertLess(
            timer.index('endpoint_trim_continuity_.note_feedback('),
            timer.index('endpoint_trim_continuity_.request_correction('),
        )
        self.assertLess(
            timer.index('endpoint_trim_continuity_.request_correction('),
            timer.index('endpoint_trim_stream_target('),
        )
        self.assertNotIn('retry_stalled_endpoint_feedback_trim', timer)

    def test_driver_endpoint_trim_has_task_scoped_expiring_lease(self):
        source = DRIVER_SOURCE.read_text()
        header = DRIVER_HEADER.read_text()
        service = source.split(
            'bool AliciaDDriverNode::set_task_endpoint_precision_callback', 1
        )[1].split('void AliciaDDriverNode::clear_retained_command_state', 1)[0]
        timer = source.split(
            'void AliciaDDriverNode::send_command_timer_callback', 1
        )[1].split('void AliciaDDriverNode::process_serial_data_callback', 1)[0]

        self.assertIn('#include "std_srvs/SetBool.h"', header)
        self.assertIn('set_task_endpoint_precision_callback', header)
        self.assertIn('task_endpoint_precision_service_ = pnh_.advertiseService(', source)
        self.assertIn('"set_task_endpoint_precision",', source)
        for member in (
            'endpoint_feedback_trim_task_lease_active_',
            'endpoint_feedback_trim_task_lease_timeout_sec_',
            'endpoint_feedback_trim_task_lease_reference_',
        ):
            self.assertIn(member, header)
        self.assertIn('endpoint_trim_continuity_.request_release(', service)
        self.assertIn('endpoint_trim_command_order_.record_release(', service)
        self.assertLess(
            service.index('endpoint_trim_continuity_.request_release('),
            service.index('endpoint_trim_command_order_.record_release('),
        )
        self.assertIn('EndpointTrimReleaseStatus::PENDING', service)
        self.assertIn('EndpointTrimReleaseStatus::COMPLETED', service)
        self.assertNotIn('endpoint_feedback_trim_offsets_', service)

        expiry_start = timer.index(
            'endpoint_feedback_trim_task_lease_expired = true;'
        )
        expiry_end = timer.index('// Lease-expiry release request end', expiry_start)
        expiry = timer[expiry_start:expiry_end]
        self.assertIn('endpoint_trim_continuity_.request_release(', expiry)
        self.assertIn('endpoint_trim_command_order_.record_release(', expiry)
        self.assertLess(
            expiry.index('endpoint_trim_continuity_.request_release('),
            expiry.index('endpoint_trim_command_order_.record_release('),
        )
        self.assertNotIn('endpoint_feedback_trim_offsets_', expiry)

    def test_committed_gui_sources_handoff_and_holdoff_are_self_contained(self):
        source = DRIVER_SOURCE.read_text()
        admission = ENDPOINT_TRIM_ADMISSION.read_text()
        callback = source.split(
            'void AliciaDDriverNode::joint_command_callback', 1
        )[1].split('void AliciaDDriverNode::send_command_timer_callback', 1)[0]

        self.assertIn('msg->header.frame_id == "gui_direct"', callback)
        self.assertIn('msg->header.frame_id == "gui_direct_sync"', callback)
        self.assertIn('EndpointTrimCommandSource::GUI_DIRECT_EDIT', callback)
        self.assertIn('EndpointTrimCommandSource::GUI_DIRECT_SYNC', callback)
        self.assertIn('EndpointTrimCommandSource::TASK_CONTROLLER', callback)
        self.assertIn(
            'endpoint_trim_command_source !=\n'
            '            EndpointTrimCommandSource::TASK_CONTROLLER',
            callback,
        )
        observe = callback.index(
            'endpoint_trim_command_order_.observe_upstream_command('
        )
        gui_branch = callback.index('if (endpoint_trim_explicit_gui_command)')
        handoff = callback.index('endpoint_trim_continuity_.explicit_gui_handoff(')
        applied = callback.index('endpoint_trim_command_order_.mark_command_applied();')
        self.assertLess(observe, handoff)
        self.assertLess(gui_branch, handoff)
        self.assertLess(handoff, applied)
        self.assertIn('endpoint_feedback_trim_task_lease_active_ = false;', callback)

        self.assertIn('double gui_task_holdoff_sec = 0.25', admission)
        self.assertIn('now_sec <= gui_task_holdoff_until_sec_', admission)
        sync_branch = admission.index(
            'source == EndpointTrimCommandSource::GUI_DIRECT_SYNC'
        )
        edit_branch = admission.index(
            'source == EndpointTrimCommandSource::GUI_DIRECT_EDIT',
            sync_branch,
        )
        self.assertIn(
            'gui_task_holdoff_active_ = false;',
            admission[sync_branch:edit_branch],
        )


if __name__ == '__main__':
    unittest.main()
