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

from gui import theme
from gui.main_gui import MainWindow
from gui.widgets import perception_widget as perception_widget_module
from gui.widgets.grasp6d_control_widget import Grasp6DControlWidget


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
        self.assertEqual(tabs.count(), 8)
        expected_tabs = (
            '总览监控',
            '关节控制',
            '笛卡尔控制',
            'TCP标定',
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
