#!/usr/bin/env python3
import os
import signal
import sys
import threading
from collections import namedtuple
from pathlib import Path


DatasetCategory = namedtuple('DatasetCategory', ['key', 'label', 'dirname'])

DATASET_CATEGORIES = (
    DatasetCategory('positive', '有纸盒', 'positive'),
    DatasetCategory('negative', '无纸盒', 'negative'),
    DatasetCategory('low_sample', '低样本', 'low_sample'),
)


def expanded_path(path):
    return Path(os.path.expandvars(str(path))).expanduser()


def ensure_category_dirs(output_root):
    root = expanded_path(output_root)
    paths = {}
    for category in DATASET_CATEGORIES:
        directory = root / category.dirname
        directory.mkdir(parents=True, exist_ok=True)
        paths[category.key] = directory
    return paths


def next_sequence_path(directory):
    directory = expanded_path(directory)
    largest = 0
    if directory.exists():
        for path in directory.glob('*.png'):
            if not path.stem.isdigit():
                continue
            largest = max(largest, int(path.stem))
    return directory / ('%06d.png' % (largest + 1))


try:
    import cv2
except Exception:
    cv2 = None

try:
    import rospy
    from sensor_msgs.msg import Image
except Exception:
    rospy = None
    Image = None

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except Exception:
    QtCore = None
    QtGui = None
    QtWidgets = None


def _missing_runtime_dependencies():
    missing = []
    if rospy is None or Image is None:
        missing.append('rospy/sensor_msgs')
    if CvBridge is None:
        missing.append('cv_bridge')
    if cv2 is None:
        missing.append('cv2')
    if QtCore is None or QtGui is None or QtWidgets is None:
        missing.append('PyQt5')
    return missing


if QtWidgets is not None:
    class RgbDatasetCollectorWindow(QtWidgets.QWidget):
        frame_signal = QtCore.pyqtSignal(object)
        status_signal = QtCore.pyqtSignal(str)

        def __init__(self, color_topic, output_root):
            super().__init__()
            self.color_topic = str(color_topic)
            self.output_root = expanded_path(output_root)
            self.category_dirs = ensure_category_dirs(self.output_root)
            self.bridge = CvBridge()
            self._lock = threading.Lock()
            self._latest_bgr = None
            self._last_pixmap = None
            self._saved_counts = self._initial_saved_counts()
            self._subscriber = rospy.Subscriber(
                self.color_topic,
                Image,
                self._image_callback,
                queue_size=1,
                buff_size=2 ** 24,
                tcp_nodelay=True,
            )
            self._build_ui()
            self.frame_signal.connect(self._update_preview)
            self.status_signal.connect(self._set_status)

            self._ros_timer = QtCore.QTimer(self)
            self._ros_timer.timeout.connect(self._close_if_ros_shutdown)
            self._ros_timer.start(250)

        def _build_ui(self):
            self.setWindowTitle('RealSense RGB 原图采集')
            self.resize(900, 680)

            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(12, 12, 12, 12)
            layout.setSpacing(10)

            self.preview = QtWidgets.QLabel('等待 RGB 图像...')
            self.preview.setAlignment(QtCore.Qt.AlignCenter)
            self.preview.setMinimumSize(640, 480)
            self.preview.setStyleSheet(
                'QLabel { background: #10141d; color: #d7e4ff; border: 1px solid #2e8bbd; }'
            )
            layout.addWidget(self.preview, 1)

            button_row = QtWidgets.QHBoxLayout()
            button_row.setSpacing(8)
            self.buttons = {}
            for category in DATASET_CATEGORIES:
                button = QtWidgets.QPushButton(category.label)
                button.setMinimumHeight(42)
                button.clicked.connect(lambda _checked=False, key=category.key: self._save_category(key))
                button_row.addWidget(button, 1)
                self.buttons[category.key] = button
            layout.addLayout(button_row)

            self.counts_label = QtWidgets.QLabel('')
            self.counts_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            layout.addWidget(self.counts_label)

            self.paths_label = QtWidgets.QLabel(self._paths_text())
            self.paths_label.setWordWrap(True)
            self.paths_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            layout.addWidget(self.paths_label)

            self.status = QtWidgets.QLabel('订阅 %s，等待第一帧...' % self.color_topic)
            self.status.setWordWrap(True)
            self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            layout.addWidget(self.status)

            self._refresh_counts_label()

        def _paths_text(self):
            parts = []
            for category in DATASET_CATEGORIES:
                parts.append('%s: %s' % (category.label, self.category_dirs[category.key]))
            return '保存路径 | ' + ' | '.join(parts)

        def _initial_saved_counts(self):
            counts = {}
            for category in DATASET_CATEGORIES:
                next_path = next_sequence_path(self.category_dirs[category.key])
                counts[category.key] = max(0, int(next_path.stem) - 1)
            return counts

        def _refresh_counts_label(self):
            labels = []
            for category in DATASET_CATEGORIES:
                labels.append('%s %d 张' % (category.label, self._saved_counts.get(category.key, 0)))
            self.counts_label.setText(' | '.join(labels))

        def _image_callback(self, msg):
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                bgr = bgr.copy()
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            except Exception as exc:
                self.status_signal.emit('图像转换失败: %s' % exc)
                return
            with self._lock:
                self._latest_bgr = bgr
            self.frame_signal.emit(rgb)

        def _update_preview(self, rgb):
            height, width, channels = rgb.shape
            qimage = QtGui.QImage(
                rgb.data,
                width,
                height,
                channels * width,
                QtGui.QImage.Format_RGB888,
            )
            self._last_pixmap = QtGui.QPixmap.fromImage(qimage.copy())
            self._render_preview()
            self._set_status('RGB %d x %d | 订阅 %s' % (width, height, self.color_topic))

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self._render_preview()

        def _render_preview(self):
            if self._last_pixmap is None:
                return
            self.preview.setPixmap(
                self._last_pixmap.scaled(
                    self.preview.size(),
                    QtCore.Qt.KeepAspectRatio,
                    QtCore.Qt.FastTransformation,
                )
            )

        def _save_category(self, key):
            with self._lock:
                frame = None if self._latest_bgr is None else self._latest_bgr.copy()
            if frame is None:
                self._set_status('尚未收到 RGB 图像')
                return
            directory = self.category_dirs[key]
            path = next_sequence_path(directory)
            try:
                ok = cv2.imwrite(str(path), frame)
                if not ok:
                    raise RuntimeError('cv2.imwrite returned false')
            except Exception as exc:
                self._set_status('保存失败 %s: %s' % (path, exc))
                return
            self._saved_counts[key] = int(path.stem)
            self._refresh_counts_label()
            label = next(category.label for category in DATASET_CATEGORIES if category.key == key)
            self._set_status('已保存 %s: %s' % (label, path))

        def _set_status(self, text):
            self.status.setText(str(text))

        def _close_if_ros_shutdown(self):
            if rospy.is_shutdown():
                self.close()

        def closeEvent(self, event):
            try:
                self._subscriber.unregister()
            except Exception:
                pass
            rospy.signal_shutdown('RGB dataset collector closed')
            super().closeEvent(event)
else:
    RgbDatasetCollectorWindow = None


def main():
    missing = _missing_runtime_dependencies()
    if missing:
        sys.stderr.write('Missing runtime dependencies: %s\n' % ', '.join(missing))
        return 2

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.init_node('rgb_dataset_collector_gui', anonymous=False)
    default_topic = rospy.get_param('/camera/color_topic', '/supervisor/camera/color/image_raw')
    color_topic = rospy.get_param('~color_topic', default_topic)
    output_root = rospy.get_param('~output_root', '~/carton_dataset/raw_rgb')

    app = QtWidgets.QApplication(sys.argv)
    window = RgbDatasetCollectorWindow(color_topic, output_root)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
