from PyQt5 import QtWidgets, QtCore
import rospy
from alicia_flexible_grasp_supervisor.msg import TactileState
from gui.theme import metric_chip, panel, set_monospace

class TactileWidget(QtWidgets.QWidget):
    sig = QtCore.pyqtSignal(object)
    def __init__(self, topic='/tactile/state'):
        super().__init__()
        self._alive = True
        self._subscriber = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        frame, body = panel('电子皮肤力反馈')
        layout.addWidget(frame)

        metrics = QtWidgets.QGridLayout()
        metrics.setSpacing(8)
        self.total_chip = metric_chip('总力 -- mN', accent=True)
        self.left_chip = metric_chip('左侧 --')
        self.right_chip = metric_chip('右侧 --')
        self.diff_chip = metric_chip('差值 --')
        self.contact_chip = metric_chip('接触 --')
        self.slip_chip = metric_chip('滑移 --')
        chips = [
            self.total_chip, self.left_chip, self.right_chip,
            self.diff_chip, self.contact_chip, self.slip_chip,
        ]
        for index, chip in enumerate(chips):
            metrics.addWidget(chip, index // 3, index % 3)
        body.addLayout(metrics)

        self.grid = QtWidgets.QTextEdit()
        self.grid.setReadOnly(True)
        self.grid.setMaximumHeight(152)
        self.grid.setPlaceholderText('等待触觉阵列数据...')
        set_monospace(self.grid)
        body.addWidget(self.grid)

        self.sig.connect(self.update_state)
        self._subscriber = rospy.Subscriber(topic, TactileState, self._emit_if_alive, queue_size=1)
        self.destroyed.connect(lambda *_: self._shutdown_ros())

    def _emit_if_alive(self, msg):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.sig.emit(msg)
        except RuntimeError:
            self._shutdown_ros()

    def _shutdown_ros(self):
        self._alive = False
        subscriber = self.__dict__.get('_subscriber', None)
        if subscriber is not None and not self.__dict__.get('_subscriber_unregistered', False):
            try:
                subscriber.unregister()
            except Exception:
                pass
            self._subscriber_unregistered = True

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)

    def update_state(self, msg):
        self.total_chip.setText('总力 %.1f mN' % msg.total_grip_force_mn)
        self.left_chip.setText('左侧 %.1f' % msg.left.total_force_mn)
        self.right_chip.setText('右侧 %.1f' % msg.right.total_force_mn)
        self.diff_chip.setText('差值 %.1f' % msg.force_diff_mn)
        self.contact_chip.setText('接触 %s' % ('是' if msg.object_grasped else '否'))
        self.slip_chip.setText('滑移 %s' % ('是' if msg.slip_detected else '否'))

        vals = list(msg.left.values)[:30]
        rows=[]
        for i in range(0, len(vals), 6):
            rows.append(' '.join('%5.0f'%v for v in vals[i:i+6]))
        self.grid.setPlainText('\n'.join(rows))
