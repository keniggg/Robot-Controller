from PyQt5 import QtWidgets, QtCore
import rospy
from alicia_flexible_grasp_supervisor.srv import CartesianJog
from gui.widgets.camera_widget import CameraWidget
from gui.theme import panel

class CartesianControlWidget(QtWidgets.QWidget):
    def __init__(self, color_topic=None, depth_topic=None):
        super().__init__()
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(14)
        controls = QtWidgets.QVBoxLayout()
        controls.setSpacing(0)
        frame, body = panel('笛卡尔空间控制')
        controls.addWidget(frame)

        step_row = QtWidgets.QHBoxLayout()
        step_label = QtWidgets.QLabel('笛卡尔点动步长')
        self.step=QtWidgets.QDoubleSpinBox()
        self.step.setDecimals(3)
        self.step.setSingleStep(0.001)
        self.step.setValue(0.005)
        self.step.setSuffix(' m')
        self.step.setMinimumWidth(160)
        step_row.addWidget(step_label)
        step_row.addWidget(self.step)
        step_row.addStretch(1)
        body.addLayout(step_row)

        grid=QtWidgets.QGridLayout()
        grid.setSpacing(10)
        body.addLayout(grid)
        buttons=[('X+',0.005,0,0),('X-',-0.005,0,0),('Y+',0,0.005,0),('Y-',0,-0.005,0),('Z+',0,0,0.005),('Z-',0,0,-0.005)]
        for i,(txt,dx,dy,dz) in enumerate(buttons):
            b=QtWidgets.QPushButton(txt)
            b.setObjectName('AxisButton')
            b.setCursor(QtCore.Qt.PointingHandCursor)
            b.clicked.connect(lambda _,dx=dx,dy=dy,dz=dz:self.jog(dx,dy,dz))
            grid.addWidget(b,i//2,i%2)

        self.status=QtWidgets.QLabel('等待操作')
        self.status.setObjectName('StateBanner')
        self.status.setWordWrap(True)
        body.addWidget(self.status)
        controls.addStretch(1)
        layout.addLayout(controls, 4)
        if color_topic and depth_topic:
            self.camera_preview = CameraWidget(color_topic, depth_topic, compact=True, default_mode='color')
            layout.addWidget(self.camera_preview, 3)

    def jog(self,dx,dy,dz):
        scale=self.step.value()/0.005
        try:
            rospy.wait_for_service('/supervisor/cartesian_jog', timeout=1.0)
            srv=rospy.ServiceProxy('/supervisor/cartesian_jog', CartesianJog)
            r=srv(dx*scale,dy*scale,dz*scale,0,0,0,True)
            self.status.setText(r.message)
        except Exception as e:
            self.status.setText(str(e))
