from PyQt5 import QtWidgets, QtCore, QtGui
import rospy
from sensor_msgs.msg import JointState
from gui.theme import metric_chip, panel

class RobotStateWidget(QtWidgets.QWidget):
    sig = QtCore.pyqtSignal(object)
    def __init__(self, topic='/joint_states'):
        super().__init__()
        self._alive = True
        self._subscriber = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        frame, body = panel('机械臂关节状态')
        layout.addWidget(frame)

        stats = QtWidgets.QHBoxLayout()
        stats.setSpacing(8)
        self.count_chip = metric_chip('关节 0', accent=True)
        self.source_chip = metric_chip('JointState')
        stats.addWidget(self.count_chip)
        stats.addWidget(self.source_chip)
        body.addLayout(stats)

        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(['关节', '位置 rad'])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setFocusPolicy(QtCore.Qt.NoFocus)
        self.table.setMinimumHeight(184)
        body.addWidget(self.table)

        self.sig.connect(self.update_state)
        self._subscriber = rospy.Subscriber(topic, JointState, self._emit_if_alive, queue_size=1)
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

    def update_state(self,msg):
        pairs = list(zip(msg.name, msg.position))
        self.table.setRowCount(len(pairs))
        mono = QtGui.QFont('JetBrains Mono')
        mono.setStyleHint(QtGui.QFont.Monospace)

        for row, (name, position) in enumerate(pairs):
            name_item = QtWidgets.QTableWidgetItem(str(name))
            value_item = QtWidgets.QTableWidgetItem('%.4f' % position)
            value_item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            value_item.setFont(mono)
            self.table.setItem(row, 0, name_item)
            self.table.setItem(row, 1, value_item)

        self.table.resizeColumnsToContents()
        self.count_chip.setText('关节 %d' % len(pairs))
