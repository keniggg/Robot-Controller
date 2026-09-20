#!/usr/bin/env python3
import os
import pathlib
import sys
import unittest
from unittest import mock


os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from PyQt5 import QtCore, QtGui, QtTest, QtWidgets
import rospy
from sensor_msgs.msg import JointState

from gui import theme
from gui.main_gui import MainWindow
from gui.widgets import perception_widget as perception_widget_module
from gui.widgets.grasp6d_control_widget import Grasp6DControlWidget
from gui.widgets.handeye_calibration_widget import (
    HandeyeCalibrationWidget,
    OnDemandRgbView,
)


class InertRosEndpoint:
    def publish(self, *_args, **_kwargs):
        return None

    def unregister(self):
        return None

    def get_num_connections(self):
        return 0


class GuiLayoutContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication(['gui-layout-contract'])
        theme.apply_app_theme(cls.app)

    def setUp(self):
        self._patchers = [
            mock.patch.object(
                rospy,
                'get_param',
                side_effect=lambda _name, default=None: default,
            ),
            # Constructing the direct-control checkbox persists its value.
            # A layout test must never change the live robot's control owner.
            mock.patch.object(rospy, 'set_param'),
            mock.patch.object(
                rospy,
                'Publisher',
                side_effect=lambda *_args, **_kwargs: InertRosEndpoint(),
            ),
            mock.patch.object(
                rospy,
                'Subscriber',
                side_effect=lambda *_args, **_kwargs: InertRosEndpoint(),
            ),
            mock.patch.object(perception_widget_module, 'tf2_ros', None),
        ]
        for patcher in self._patchers:
            patcher.start()

    def tearDown(self):
        for patcher in reversed(self._patchers):
            patcher.stop()

    @staticmethod
    def _button_content_rect(button, pressed):
        option = QtWidgets.QStyleOptionButton()
        option.initFrom(button)
        option.text = button.text()
        if pressed:
            option.state |= QtWidgets.QStyle.State_Sunken
        else:
            option.state &= ~QtWidgets.QStyle.State_Sunken
        return button.style().subElementRect(
            QtWidgets.QStyle.SE_PushButtonContents,
            option,
            button,
        )

    @staticmethod
    def _pixel_lightness(image, x, y):
        color = image.pixelColor(int(x), int(y))
        return QtGui.qGray(color.rgb())

    def test_primary_button_press_is_brighter_and_visibly_lower(self):
        button = QtWidgets.QPushButton('执行动作')
        button.setObjectName('PrimaryButton')
        button.setFixedSize(190, 52)
        button.show()
        self.app.processEvents()

        normal = button.grab().toImage()
        normal_content = self._button_content_rect(button, pressed=False)
        normal_lightness = self._pixel_lightness(normal, 24, 18)

        QtTest.QTest.mousePress(button, QtCore.Qt.LeftButton)
        self.app.processEvents()
        pressed = button.grab().toImage()
        pressed_content = self._button_content_rect(button, pressed=True)
        pressed_lightness = self._pixel_lightness(pressed, 24, 18)

        self.assertNotEqual(pressed, normal)
        self.assertGreater(pressed_lightness, normal_lightness)
        self.assertGreater(pressed_content.top(), normal_content.top())

        QtTest.QTest.mouseRelease(button, QtCore.Qt.LeftButton)
        self.app.processEvents()
        released = button.grab().toImage()
        self.assertEqual(
            self._pixel_lightness(released, 24, 18),
            normal_lightness,
        )
        button.close()

    def test_disabled_emphasis_buttons_lose_their_active_fill(self):
        for object_name in ('PrimaryButton', 'DangerButton'):
            with self.subTest(object_name=object_name):
                button = QtWidgets.QPushButton('执行动作')
                button.setObjectName(object_name)
                button.setFixedSize(190, 52)
                button.show()
                self.app.processEvents()
                enabled = button.grab().toImage()
                enabled_lightness = self._pixel_lightness(
                    enabled,
                    24,
                    18,
                )

                button.setEnabled(False)
                self.app.processEvents()
                disabled = button.grab().toImage()
                disabled_lightness = self._pixel_lightness(
                    disabled,
                    24,
                    18,
                )

                self.assertNotEqual(disabled, enabled)
                self.assertLess(disabled_lightness, enabled_lightness)
                button.close()

    def test_checkable_group_indicator_is_large_and_clear_of_title(self):
        group = QtWidgets.QGroupBox('高级视觉参数')
        group.setCheckable(True)
        group.setChecked(False)
        group.resize(360, 120)
        group.show()
        self.app.processEvents()

        option = QtWidgets.QStyleOptionGroupBox()
        option.initFrom(group)
        option.text = group.title()
        option.lineWidth = 1
        option.subControls = (
            QtWidgets.QStyle.SC_GroupBoxFrame
            | QtWidgets.QStyle.SC_GroupBoxLabel
            | QtWidgets.QStyle.SC_GroupBoxCheckBox
        )
        indicator = group.style().subControlRect(
            QtWidgets.QStyle.CC_GroupBox,
            option,
            QtWidgets.QStyle.SC_GroupBoxCheckBox,
            group,
        )
        title = group.style().subControlRect(
            QtWidgets.QStyle.CC_GroupBox,
            option,
            QtWidgets.QStyle.SC_GroupBoxLabel,
            group,
        )

        self.assertGreaterEqual(indicator.width(), 17)
        self.assertGreaterEqual(indicator.height(), 17)
        self.assertFalse(indicator.intersects(title))
        group.close()

    def test_vertical_scroll_area_owns_child_without_horizontal_scroll(self):
        factory = getattr(theme, 'make_vertical_scroll_area', None)
        self.assertTrue(
            callable(factory),
            'theme must expose make_vertical_scroll_area',
        )
        child = QtWidgets.QWidget()
        scroll = factory(child, 'ProbeScroll')

        self.assertEqual(scroll.objectName(), 'ProbeScroll')
        self.assertIs(scroll.widget(), child)
        self.assertTrue(scroll.widgetResizable())
        self.assertEqual(
            scroll.horizontalScrollBarPolicy(),
            QtCore.Qt.ScrollBarAlwaysOff,
        )
        self.assertEqual(
            scroll.verticalScrollBarPolicy(),
            QtCore.Qt.ScrollBarAsNeeded,
        )
        self.assertEqual(scroll.frameShape(), QtWidgets.QFrame.NoFrame)
        scroll.close()

    def test_minimum_window_contains_scrolls_tabs_and_compact_action_grid(self):
        window = MainWindow()
        window.resize(1120, 720)
        window.show()
        self.app.processEvents()

        overview_scroll = window.findChild(
            QtWidgets.QScrollArea,
            'OverviewStatusScroll',
        )
        perception_scroll = window.findChild(
            QtWidgets.QScrollArea,
            'PerceptionControlsScroll',
        )
        self.assertIsNotNone(overview_scroll)
        self.assertIsNotNone(perception_scroll)

        tabs = window.findChild(QtWidgets.QTabWidget)
        self.assertIsNotNone(tabs)
        self.assertEqual(tabs.count(), 9)
        expected_tabs = (
            '总览监控',
            '关节控制',
            '笛卡尔控制',
            'TCP标定',
            '手眼标定',
            '目标识别',
            '6D抓取',
            '电子皮肤',
            '日志',
        )
        self.assertEqual(
            tuple(tabs.tabText(index) for index in range(tabs.count())),
            expected_tabs,
        )
        tab_bar = tabs.tabBar()
        self.assertLessEqual(
            tab_bar.tabRect(tabs.count() - 1).right(),
            tab_bar.rect().right(),
        )
        visible_scroll_buttons = [
            button
            for button in tab_bar.findChildren(QtWidgets.QToolButton)
            if button.isVisible()
        ]
        self.assertEqual(visible_scroll_buttons, [])

        handeye = window.findChild(HandeyeCalibrationWidget)
        self.assertIsNotNone(handeye)
        tabs.setCurrentIndex(4)
        self.app.processEvents()
        image_panel = handeye.findChild(
            QtWidgets.QFrame,
            'HandeyeImagePanel',
        )
        sampling_panel = handeye.findChild(
            QtWidgets.QFrame,
            'HandeyeSamplingPanel',
        )
        self.assertIsNotNone(image_panel)
        self.assertIsNotNone(sampling_panel)
        self.assertTrue(handeye.joint_control.isVisible())
        self.assertTrue(image_panel.isVisible())
        self.assertTrue(sampling_panel.isVisible())
        self.assertGreaterEqual(handeye.joint_control.width(), 500)
        self.assertGreaterEqual(image_panel.width(), 430)
        self.assertEqual(handeye.findChildren(QtWidgets.QScrollArea), [])
        feedback_angles = handeye.findChildren(
            QtWidgets.QLabel,
            'JointFeedbackAngle',
        )
        self.assertEqual(len(feedback_angles), 6)
        feedback = JointState()
        feedback.name = [
            'Joint1',
            'Joint2',
            'Joint3',
            'Joint4',
            'Joint5',
            'Joint6',
        ]
        feedback.position = [0.0, 0.1, -0.2, 0.3, -0.4, 0.5]
        handeye.joint_control.update_current_state(feedback)
        self.assertEqual(feedback_angles[0].text(), '+0.00°')
        self.assertEqual(feedback_angles[1].text(), '+5.73°')
        self.assertEqual(feedback_angles[2].text(), '-11.46°')
        image_view = handeye.findChild(OnDemandRgbView)
        self.assertIsNotNone(image_view)
        self.assertFalse(image_view.streaming)
        self.assertFalse(image_view.isVisible())
        image_selector = handeye.findChild(
            QtWidgets.QComboBox,
            'HandeyeImageModeSelector',
        )
        self.assertIsNotNone(image_selector)
        self.assertEqual(image_selector.count(), 2)
        self.assertEqual(image_selector.currentData(), 'off')
        image_selector.setCurrentIndex(image_selector.findData('on'))
        self.app.processEvents()
        self.assertTrue(image_view.streaming)
        self.assertTrue(image_view.isVisible())
        self.assertTrue(handeye.joint_control.isVisible())
        self.assertTrue(sampling_panel.isVisible())
        for widget in (
            handeye.joint_control,
            image_panel,
            sampling_panel,
            image_selector,
            handeye.stable_btn,
            handeye.take_sample_btn,
            handeye.sample_table,
            handeye.calibration_output,
        ):
            with self.subTest(handeye_widget=widget.objectName() or type(widget).__name__):
                top_left = widget.mapTo(handeye, QtCore.QPoint(0, 0))
                bottom_right = widget.mapTo(
                    handeye,
                    QtCore.QPoint(widget.width() - 1, widget.height() - 1),
                )
                self.assertTrue(handeye.rect().contains(top_left))
                self.assertTrue(handeye.rect().contains(bottom_right))
        image_selector.setCurrentIndex(image_selector.findData('off'))
        self.app.processEvents()
        self.assertFalse(image_view.streaming)
        self.assertFalse(image_view.isVisible())
        compact = next(
            widget
            for widget in window.findChildren(Grasp6DControlWidget)
            if widget._compact
        )
        grid = compact.findChild(
            QtWidgets.QGridLayout,
            'CompactActionGrid',
        )
        self.assertIsNotNone(grid)
        self.assertIs(grid.itemAtPosition(0, 0).widget(), compact.check_btn)
        self.assertIs(
            grid.itemAtPosition(0, 1).widget(),
            compact.request_plan_btn,
        )
        self.assertIs(grid.itemAtPosition(1, 0).widget(), compact.execute_btn)
        self.assertIs(grid.itemAtPosition(1, 1).widget(), compact.stop_btn)
        for button in (
            compact.check_btn,
            compact.request_plan_btn,
            compact.execute_btn,
            compact.stop_btn,
        ):
            self.assertGreaterEqual(
                button.contentsRect().width(),
                button.fontMetrics().horizontalAdvance(button.text()),
            )

        window.close()
        window.deleteLater()
        self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
