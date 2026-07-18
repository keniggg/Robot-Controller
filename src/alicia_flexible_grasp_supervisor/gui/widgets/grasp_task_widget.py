from PyQt5 import QtWidgets, QtCore
import rospy
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StopGrasp
from gui.theme import panel

class GraspTaskWidget(QtWidgets.QWidget):
    sig=QtCore.pyqtSignal(object)
    def __init__(self, topic='/grasp/state'):
        super().__init__()
        self._alive = True
        self._subscriber = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        frame, body = panel('自主柔顺抓取')
        layout.addWidget(frame)

        self.state=QtWidgets.QLabel('抓取状态：IDLE')
        self.state.setObjectName('StateBanner')
        self.state.setWordWrap(True)
        self.state.setMinimumHeight(58)

        actions = QtWidgets.QHBoxLayout()
        actions.setSpacing(10)
        start=QtWidgets.QPushButton('开始自主柔顺抓取')
        start.setObjectName('PrimaryButton')
        stop=QtWidgets.QPushButton('停止')
        stop.setObjectName('DangerButton')
        start.clicked.connect(self.start); stop.clicked.connect(self.stop)
        actions.addWidget(start, 2)
        actions.addWidget(stop, 1)
        body.addWidget(self.state)
        body.addLayout(actions)

        self.sig.connect(self.update)
        self._subscriber = rospy.Subscriber(topic, GraspState, self._emit_if_alive, queue_size=1)
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

    def update(self,msg):
        self.state.setText('抓取状态：%s | %s'%(msg.state,msg.message))

    def start(self):
        try:
            rospy.wait_for_service('/grasp/start', timeout=1)
            r=rospy.ServiceProxy('/grasp/start', StartGrasp)(execute=True)
            self.state.setText('启动结果：%s %s'%(r.success,r.message))
        except Exception as e:
            self.state.setText(str(e))

    def stop(self):
        try:
            rospy.wait_for_service('/grasp/stop', timeout=1)
            r=rospy.ServiceProxy('/grasp/stop', StopGrasp)(False)
            self.state.setText('停止结果：%s'%r.message)
        except Exception as e:
            self.state.setText(str(e))
