from PyQt5 import QtWidgets
from gui.theme import panel, set_monospace

class LogWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        frame, body = panel('系统日志')
        layout.addWidget(frame)

        self.text=QtWidgets.QTextEdit()
        self.text.setReadOnly(True)
        self.text.setPlainText('ROS 日志请同时查看终端输出。')
        set_monospace(self.text)
        body.addWidget(self.text, 1)
