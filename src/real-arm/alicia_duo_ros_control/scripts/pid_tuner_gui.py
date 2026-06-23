#!/usr/bin/env python

import rospy
import sys
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QScrollArea, QGroupBox, QFormLayout, QTextEdit, QApplication
from python_qt_binding.QtCore import Qt, QTimer # Import QTimer
from functools import partial

from controller_manager_msgs.srv import (
    LoadController, UnloadController, SwitchController, SwitchControllerRequest
)

# --- Configuration ---
CONTROLLER_NAME = "arm_pos_controller"
# IMPORTANT: Make sure these match your controller's YAML configuration and URDF
JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
DEFAULT_P = 0.0
DEFAULT_I = 0.1
DEFAULT_D = 1
DEFAULT_I_CLAMP = 10.0
# ---------------------

class PIDTunerWidget(QWidget):
    def __init__(self):
        super(PIDTunerWidget, self).__init__()
        self.setObjectName('PIDTunerWidget')

        self.main_layout = QVBoxLayout(self)

        # --- Status Log ---
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(100) # Limit height
        self.main_layout.addWidget(QLabel("Status Log:"))
        self.main_layout.addWidget(self.status_log)

        # --- Scroll Area for Joint Controls ---
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.joints_layout = QVBoxLayout(scroll_widget)
        scroll_area.setWidget(scroll_widget)
        self.main_layout.addWidget(scroll_area)

        self.joint_pid_widgets = {} # To store P, I, D, I_clamp spinboxes for each joint

        for joint_name in JOINT_NAMES:
            group_box = QGroupBox(joint_name)
            form_layout = QFormLayout()

            p_spinbox = QDoubleSpinBox()
            p_spinbox.setRange(0, 10000.0)
            p_spinbox.setDecimals(2)
            p_spinbox.setValue(DEFAULT_P)
            form_layout.addRow(f"P ({joint_name}):", p_spinbox)

            i_spinbox = QDoubleSpinBox()
            i_spinbox.setRange(0, 1000.0)
            i_spinbox.setDecimals(3)
            i_spinbox.setValue(DEFAULT_I)
            form_layout.addRow(f"I ({joint_name}):", i_spinbox)

            d_spinbox = QDoubleSpinBox()
            d_spinbox.setRange(0, 1000.0)
            d_spinbox.setDecimals(2)
            d_spinbox.setValue(DEFAULT_D)
            form_layout.addRow(f"D ({joint_name}):", d_spinbox)

            i_clamp_spinbox = QDoubleSpinBox()
            i_clamp_spinbox.setRange(0, 1000.0) # Or -1000 to 1000 if you allow negative i_clamp
            i_clamp_spinbox.setDecimals(2)
            i_clamp_spinbox.setValue(DEFAULT_I_CLAMP)
            form_layout.addRow(f"I_clamp ({joint_name}):", i_clamp_spinbox)

            apply_button = QPushButton(f"Apply to {joint_name}")
            # Use partial to pass joint_name and its specific spinboxes
            apply_button.clicked.connect(
                partial(self._handle_apply_single_joint, joint_name,
                        p_spinbox, i_spinbox, d_spinbox, i_clamp_spinbox)
            )
            form_layout.addRow(apply_button)

            group_box.setLayout(form_layout)
            self.joints_layout.addWidget(group_box)
            self.joint_pid_widgets[joint_name] = {
                'p': p_spinbox, 'i': i_spinbox, 'd': d_spinbox, 'i_clamp': i_clamp_spinbox
            }

        # --- Global PID Settings ---
        global_group_box = QGroupBox("Apply to ALL Joints")
        global_form_layout = QFormLayout()

        self.global_p_spinbox = QDoubleSpinBox()
        self.global_p_spinbox.setRange(0, 10000.0); self.global_p_spinbox.setDecimals(2); self.global_p_spinbox.setValue(DEFAULT_P)
        global_form_layout.addRow("Global P:", self.global_p_spinbox)

        self.global_i_spinbox = QDoubleSpinBox()
        self.global_i_spinbox.setRange(0, 1000.0); self.global_i_spinbox.setDecimals(3); self.global_i_spinbox.setValue(DEFAULT_I)
        global_form_layout.addRow("Global I:", self.global_i_spinbox)

        self.global_d_spinbox = QDoubleSpinBox()
        self.global_d_spinbox.setRange(0, 1000.0); self.global_d_spinbox.setDecimals(2); self.global_d_spinbox.setValue(DEFAULT_D)
        global_form_layout.addRow("Global D:", self.global_d_spinbox)

        self.global_i_clamp_spinbox = QDoubleSpinBox()
        self.global_i_clamp_spinbox.setRange(0, 1000.0); self.global_i_clamp_spinbox.setDecimals(2); self.global_i_clamp_spinbox.setValue(DEFAULT_I_CLAMP)
        global_form_layout.addRow("Global I_clamp:", self.global_i_clamp_spinbox)

        apply_all_button = QPushButton("Apply Global PID to ALL Joints")
        apply_all_button.clicked.connect(self._handle_apply_all_joints)
        global_form_layout.addRow(apply_all_button)

        get_current_params_button = QPushButton("Get Current Params from Server")
        get_current_params_button.clicked.connect(self._handle_get_current_params)
        global_form_layout.addRow(get_current_params_button)


        global_group_box.setLayout(global_form_layout)
        self.main_layout.addWidget(global_group_box)

        self.setLayout(self.main_layout)
        self.setWindowTitle('PID Tuner')

        # Service Proxies
        self._switch_controller = None
        self._load_controller = None
        self._unload_controller = None
        self._connect_services()

        # Timer to process ROS events if running standalone
        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._process_ros_events)
        self._ros_timer.start(100) # ms

    def _process_ros_events(self):
        if rospy.is_shutdown():
            QApplication.instance().quit()

    def _log_to_gui(self, message, level="info"):
        if level == "error":
            rospy.logerr(message)
            self.status_log.append(f"<font color='red'>ERROR: {message}</font>")
        elif level == "warn":
            rospy.logwarn(message)
            self.status_log.append(f"<font color='orange'>WARN: {message}</font>")
        else:
            rospy.loginfo(message)
            self.status_log.append(message)
        self.status_log.ensureCursorVisible() # Scroll to bottom


    def _connect_services(self):
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=1.0)
            self._switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            rospy.wait_for_service('/controller_manager/load_controller', timeout=1.0)
            self._load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
            rospy.wait_for_service('/controller_manager/unload_controller', timeout=1.0)
            self._unload_controller = rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
            self._log_to_gui("Connected to controller_manager services.")
        except rospy.ROSException as e:
            self._log_to_gui(f"Failed to connect to controller_manager services: {e}", "error")
        except rospy.ServiceException as e: # Added for timeout
             self._log_to_gui(f"Service connection timed out: {e}", "error")


    def _set_rosparams(self, joint_name, p, i, d, i_clamp):
        try:
            base_param_name = f"/{CONTROLLER_NAME}/gains/{joint_name}"
            rospy.set_param(f"{base_param_name}/p", float(p))
            rospy.set_param(f"{base_param_name}/i", float(i))
            rospy.set_param(f"{base_param_name}/d", float(d))
            rospy.set_param(f"{base_param_name}/i_clamp", float(i_clamp)) # Or i_clamp_min/max
            self._log_to_gui(f"ROS Params set for {joint_name}: P={p}, I={i}, D={d}, I_clamp={i_clamp}")
            return True
        except Exception as e:
            self._log_to_gui(f"Failed to set ROS Params for {joint_name}: {e}", "error")
            return False

    def _reload_controller(self):
        if not all([self._switch_controller, self._load_controller, self._unload_controller]):
            self._log_to_gui("Controller manager services not available. Cannot reload.", "error")
            self._connect_services() # Try to reconnect
            return False
        try:
            self._log_to_gui(f"Attempting to reload controller: {CONTROLLER_NAME}")
            # 1. Stop
            resp = self._switch_controller(stop_controllers=[CONTROLLER_NAME], start_controllers=[], strictness=SwitchControllerRequest.BEST_EFFORT)
            if not resp.ok:
                self._log_to_gui(f"Failed to stop controller {CONTROLLER_NAME}. It might not be running.", "warn")
            else:
                self._log_to_gui(f"Controller {CONTROLLER_NAME} stopped.")

            # 2. Unload
            resp_unload = self._unload_controller(CONTROLLER_NAME)
            if not resp_unload.ok:
                self._log_to_gui(f"Failed to unload controller {CONTROLLER_NAME}. It might not have been loaded or stopped properly.", "warn")
            else:
                self._log_to_gui(f"Controller {CONTROLLER_NAME} unloaded.")

            # 3. Load
            resp_load = self._load_controller(CONTROLLER_NAME)
            if not resp_load.ok:
                self._log_to_gui(f"Failed to load controller {CONTROLLER_NAME}. Check parameters and controller type.", "error")
                return False
            self._log_to_gui(f"Controller {CONTROLLER_NAME} loaded.")

            # 4. Start
            resp_start = self._switch_controller(start_controllers=[CONTROLLER_NAME], stop_controllers=[], strictness=SwitchControllerRequest.STRICT)
            if not resp_start.ok:
                self._log_to_gui(f"Failed to start controller {CONTROLLER_NAME}.", "error")
                return False
            self._log_to_gui(f"Controller {CONTROLLER_NAME} started successfully with new gains.")
            return True

        except rospy.ServiceException as e:
            self._log_to_gui(f"Service call failed during controller reload: {e}", "error")
            return False
        except Exception as e:
            self._log_to_gui(f"An unexpected error occurred during controller reload: {e}", "error")
            return False

    def _handle_apply_single_joint(self, joint_name, p_spinbox, i_spinbox, d_spinbox, i_clamp_spinbox):
        p = p_spinbox.value()
        i = i_spinbox.value()
        d = d_spinbox.value()
        i_clamp = i_clamp_spinbox.value()

        self._log_to_gui(f"Applying to {joint_name}: P={p}, I={i}, D={d}, I_clamp={i_clamp}")
        if self._set_rosparams(joint_name, p, i, d, i_clamp):
            self._reload_controller()

    def _handle_apply_all_joints(self):
        p = self.global_p_spinbox.value()
        i = self.global_i_spinbox.value()
        d = self.global_d_spinbox.value()
        i_clamp = self.global_i_clamp_spinbox.value()
        self._log_to_gui(f"Applying to ALL joints: P={p}, I={i}, D={d}, I_clamp={i_clamp}")

        all_params_set = True
        for joint_name in JOINT_NAMES:
            if not self._set_rosparams(joint_name, p, i, d, i_clamp):
                all_params_set = False
                break # Stop if one fails

        if all_params_set:
            self._reload_controller()
        else:
            self._log_to_gui("Failed to set parameters for one or more joints. Controller not reloaded.", "error")

    def _handle_get_current_params(self):
        self._log_to_gui("Attempting to read current parameters from ROS Parameter Server...")
        any_read = False
        for joint_name in JOINT_NAMES:
            widgets = self.joint_pid_widgets.get(joint_name)
            if not widgets:
                continue

            base_param_name = f"/{CONTROLLER_NAME}/gains/{joint_name}"
            try:
                p_val = rospy.get_param(f"{base_param_name}/p", DEFAULT_P)
                i_val = rospy.get_param(f"{base_param_name}/i", DEFAULT_I)
                d_val = rospy.get_param(f"{base_param_name}/d", DEFAULT_D)
                i_clamp_val = rospy.get_param(f"{base_param_name}/i_clamp", DEFAULT_I_CLAMP)

                widgets['p'].setValue(p_val)
                widgets['i'].setValue(i_val)
                widgets['d'].setValue(d_val)
                widgets['i_clamp'].setValue(i_clamp_val)
                self._log_to_gui(f"Read params for {joint_name}: P={p_val}, I={i_val}, D={d_val}, I_clamp={i_clamp_val}")
                any_read = True
            except rospy.ROSException as e: # Handles non-existent params
                self._log_to_gui(f"Could not read params for {joint_name} (may not be set): {e}", "warn")
            except KeyError: # If param path is wrong or not set
                self._log_to_gui(f"Parameter for {joint_name} not found on server. Using defaults.", "warn")
                widgets['p'].setValue(DEFAULT_P)
                widgets['i'].setValue(DEFAULT_I)
                widgets['d'].setValue(DEFAULT_D)
                widgets['i_clamp'].setValue(DEFAULT_I_CLAMP)


        if any_read:
             # Also update global spinboxes if all individual reads were successful and identical
            # For simplicity, just update global from first joint or defaults
            first_joint_widgets = self.joint_pid_widgets.get(JOINT_NAMES[0])
            if first_joint_widgets:
                self.global_p_spinbox.setValue(first_joint_widgets['p'].value())
                self.global_i_spinbox.setValue(first_joint_widgets['i'].value())
                self.global_d_spinbox.setValue(first_joint_widgets['d'].value())
                self.global_i_clamp_spinbox.setValue(first_joint_widgets['i_clamp'].value())
            self._log_to_gui("GUI fields updated from parameter server values (or defaults if not found).")
        else:
            self._log_to_gui("Could not read any parameters from server. GUI reflects defaults.", "warn")


# For running as a standalone script (not an rqt plugin)
if __name__ == "__main__":
    try:
        rospy.init_node('pid_tuner_gui_standalone', anonymous=True)
    except rospy.exceptions.ROSException as e:
        print(f"Error initializing ROS node. Is roscore running? {e}")
        sys.exit(1)

    app = QApplication(sys.argv)
    pid_tuner_widget = PIDTunerWidget()
    pid_tuner_widget.show()
    sys.exit(app.exec_())

# For use as an RQT plugin, you would have a class like this:
# class PIDTunerPlugin(Plugin):
#     def __init__(self, context):
#         super(PIDTunerPlugin, self).__init__(context)
#         self.setObjectName('PIDTunerPlugin')
#         self._widget = PIDTunerWidget()
#         if context.serial_number() > 1:
#             self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
#         context.add_widget(self._widget)

#     def shutdown_plugin(self):
#         # TODO unregister all publishers here
#         pass

#     def save_settings(self, plugin_settings, instance_settings):
#         # TODO save intrinsic configuration, usually using plugin_settings.set_value() / instance_settings.set_value()
#         pass

#     def restore_settings(self, plugin_settings, instance_settings):
#         # TODO restore intrinsic configuration, usually using plugin_settings.get_value() / instance_settings.get_value()
#         pass