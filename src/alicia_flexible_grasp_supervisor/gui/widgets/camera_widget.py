from PyQt5 import QtWidgets, QtGui, QtCore
import rospy
from sensor_msgs.msg import Image
from gui.theme import metric_chip, panel
try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
except Exception:
    CvBridge = None
    cv2 = None
    np = None

class CameraWidget(QtWidgets.QWidget):
    color_signal = QtCore.pyqtSignal(object)
    depth_signal = QtCore.pyqtSignal(object)

    def __init__(self, topic='/supervisor/camera/color/image_raw', depth_topic='/supervisor/camera/depth/image_raw',
                 compact=False, default_mode='split'):
        super().__init__()
        self.topic = topic
        self.depth_topic = depth_topic
        self.compact = compact
        self.bridge = CvBridge() if CvBridge else None
        self._alive = True
        self._subscribers = []
        self._last_color_pixmap = None
        self._last_depth_pixmap = None
        self._detection_overlay = None
        self._display_mode = default_mode
        self.color_display_hz = float(rospy.get_param('/gui/camera_display_hz', 10.0))
        self.depth_display_hz = float(rospy.get_param('/gui/depth_display_hz', 15.0))
        self._color_pending = False
        self._depth_pending = False
        self._last_color_display_time = 0.0
        self._last_depth_display_time = 0.0
        self._depth_range = None
        self._depth_range_time = 0.0
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        title = '摄像头预览' if compact else 'RGB-D 视觉预览'
        frame, body = panel(title)
        layout.addWidget(frame)

        meta = QtWidgets.QHBoxLayout()
        meta.setSpacing(8)
        color_topic_chip = metric_chip('RGB 图像流')
        depth_topic_chip = metric_chip('Depth 深度流')
        color_topic_chip.setToolTip(topic)
        depth_topic_chip.setToolTip(depth_topic)
        meta.addWidget(color_topic_chip, 1)
        meta.addWidget(depth_topic_chip, 1)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem('彩色', 'color')
        self.mode.addItem('深度', 'depth')
        self.mode.addItem('彩色 + 深度', 'split')
        mode_index = max(0, self.mode.findData(default_mode))
        self.mode.setCurrentIndex(mode_index)
        self.mode.currentIndexChanged.connect(self._update_mode)
        meta.addWidget(self.mode)
        body.addLayout(meta)

        status_row = QtWidgets.QHBoxLayout()
        status_row.setSpacing(8)
        self.color_status = metric_chip('彩色等待', accent=True)
        self.depth_status = metric_chip('深度等待', accent=True)
        status_row.addWidget(self.color_status)
        status_row.addWidget(self.depth_status)
        body.addLayout(status_row)

        viewport = QtWidgets.QWidget()
        self.viewport_layout = QtWidgets.QHBoxLayout(viewport)
        self.viewport_layout.setContentsMargins(0, 0, 0, 0)
        self.viewport_layout.setSpacing(10)
        self.color_label = self._make_view_label('等待彩色图像...')
        self.depth_label = self._make_view_label('等待深度图像...')
        self.viewport_layout.addWidget(self.color_label, 1)
        self.viewport_layout.addWidget(self.depth_label, 1)
        body.addWidget(viewport, 1)

        self.color_signal.connect(self.update_color_image)
        self.depth_signal.connect(self.update_depth_image)
        self._subscribers.append(rospy.Subscriber(topic, Image, self.color_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True))
        self._subscribers.append(rospy.Subscriber(depth_topic, Image, self.depth_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True))
        self.destroyed.connect(lambda *_: self._shutdown_ros())
        self._update_mode()

    def _make_view_label(self, text):
        label = QtWidgets.QLabel(text)
        label.setAlignment(QtCore.Qt.AlignCenter)
        if self.compact:
            label.setMinimumSize(240, 190)
        else:
            label.setMinimumSize(300, 460)
        label.setObjectName('CameraFrame')
        return label

    def color_cb(self, msg):
        if self.bridge is None or cv2 is None:
            return
        if getattr(self, '_display_mode', 'split') == 'depth':
            return
        if not self._begin_frame('color', rospy.get_time()):
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.color_signal.emit(rgb)
        except Exception as exc:
            self._end_frame('color')
            rospy.logwarn_throttle(2.0, 'CameraWidget color convert failed: %s', exc)

    def depth_cb(self, msg):
        if self.bridge is None or cv2 is None or np is None:
            return
        if getattr(self, '_display_mode', 'split') == 'color':
            return
        now = rospy.get_time()
        if not self._begin_frame('depth', now):
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            rgb = self._colorize_depth(depth, now)
            self.depth_signal.emit(rgb)
        except Exception as exc:
            self._end_frame('depth')
            rospy.logwarn_throttle(2.0, 'CameraWidget depth convert failed: %s', exc)

    def update_color_image(self, rgb):
        try:
            if not self.__dict__.get('_alive', False):
                return
            if self._detection_overlay is not None:
                rgb = self._draw_detection_overlay(rgb, self._detection_overlay)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            self._last_color_pixmap = QtGui.QPixmap.fromImage(qimg.copy())
            self.color_status.setText('彩色 %d x %d' % (w, h))
            self._render_pixmaps()
        except RuntimeError:
            self._shutdown_ros()
        finally:
            self._end_frame('color')

    def update_depth_image(self, rgb):
        try:
            if not self.__dict__.get('_alive', False):
                return
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            self._last_depth_pixmap = QtGui.QPixmap.fromImage(qimg.copy())
            self.depth_status.setText('深度 %d x %d' % (w, h))
            self._render_pixmaps()
        except RuntimeError:
            self._shutdown_ros()
        finally:
            self._end_frame('depth')

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_pixmaps()

    def _update_mode(self):
        mode = self.mode.currentData() if hasattr(self, 'mode') else 'split'
        self._display_mode = mode
        self.color_label.setVisible(mode in ('color', 'split'))
        self.depth_label.setVisible(mode in ('depth', 'split'))
        self._render_pixmaps()

    def _render_pixmaps(self):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self._render_one(self.color_label, self._last_color_pixmap)
            self._render_one(self.depth_label, self._last_depth_pixmap)
        except RuntimeError:
            self._shutdown_ros()

    def _render_one(self, label, pixmap):
        if pixmap is None or not label.isVisible():
            return
        label.setPixmap(pixmap.scaled(
            label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation
        ))

    def set_detection_overlay(self, bbox=None, label='', color=(80, 255, 120)):
        try:
            if bbox is None:
                self._detection_overlay = None
            else:
                self._detection_overlay = {
                    'bbox': tuple(int(v) for v in bbox[:4]),
                    'label': str(label or ''),
                    'color': tuple(int(v) for v in color[:3]),
                }
            self._render_pixmaps()
        except RuntimeError:
            self._shutdown_ros()

    def _begin_frame(self, stream, now):
        if not self.__dict__.get('_alive', False):
            return False
        pending_attr = '_%s_pending' % stream
        if self.__dict__.get(pending_attr, False):
            return False
        hz = self.__dict__.get('%s_display_hz' % stream, 10.0)
        last_attr = '_last_%s_display_time' % stream
        last_time = self.__dict__.get(last_attr, 0.0)
        min_period = 1.0 / max(1.0, float(hz))
        if float(now) - float(last_time) < min_period:
            return False
        setattr(self, last_attr, float(now))
        setattr(self, pending_attr, True)
        return True

    def _end_frame(self, stream):
        setattr(self, '_%s_pending' % stream, False)

    def _shutdown_ros(self):
        self._alive = False
        self._color_pending = False
        self._depth_pending = False
        for subscriber in list(self.__dict__.get('_subscribers', []) ):
            try:
                subscriber.unregister()
            except Exception:
                pass

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)

    @staticmethod
    def _draw_detection_overlay(rgb, overlay):
        if overlay is None or cv2 is None:
            return rgb
        bbox = overlay.get('bbox')
        if not bbox or len(bbox) != 4:
            return rgb
        x, y, w, h = [int(v) for v in bbox]
        if w <= 0 or h <= 0:
            return rgb
        drawn = rgb.copy()
        height, width = drawn.shape[:2]
        x0 = max(0, min(width - 1, x))
        y0 = max(0, min(height - 1, y))
        x1 = max(0, min(width - 1, x + w))
        y1 = max(0, min(height - 1, y + h))
        color = tuple(int(v) for v in overlay.get('color', (80, 255, 120)))
        cv2.rectangle(drawn, (x0, y0), (x1, y1), color, 2)
        label = overlay.get('label', '')
        if label:
            text_y = max(14, y0 - 6)
            cv2.putText(drawn, label, (x0, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return drawn

    def _colorize_depth(self, depth, now):
        depth = np.asarray(depth, dtype=np.float32)
        valid = np.isfinite(depth) & (depth > 0)
        if not np.any(valid):
            return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

        if self._depth_range is None or now - self._depth_range_time > 0.5:
            sample = depth[::8, ::8]
            sample_valid = np.isfinite(sample) & (sample > 0)
            if np.any(sample_valid):
                lo, hi = np.percentile(sample[sample_valid], [2.0, 98.0])
            else:
                lo, hi = np.percentile(depth[valid], [2.0, 98.0])
            if hi <= lo:
                hi = lo + 1.0
            self._depth_range = (float(lo), float(hi))
            self._depth_range_time = now
        lo, hi = self._depth_range
        if hi <= lo:
            hi = lo + 1.0
        scaled = cv2.convertScaleAbs(depth, alpha=255.0 / (hi - lo), beta=-lo * 255.0 / (hi - lo))
        scaled[~valid] = 0
        cmap = cv2.COLORMAP_TURBO if hasattr(cv2, 'COLORMAP_TURBO') else cv2.COLORMAP_JET
        color = cv2.applyColorMap(scaled, cmap)
        color[~valid] = (0, 0, 0)
        return cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
