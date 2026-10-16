from PyQt5 import QtCore, QtGui, QtWidgets


class HudRoot(QtWidgets.QWidget):
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        rect = self.rect()

        gradient = QtGui.QLinearGradient(rect.topLeft(), rect.bottomRight())
        gradient.setColorAt(0.0, QtGui.QColor("#040711"))
        gradient.setColorAt(0.42, QtGui.QColor("#08112c"))
        gradient.setColorAt(0.74, QtGui.QColor("#12091f"))
        gradient.setColorAt(1.0, QtGui.QColor("#050610"))
        painter.fillRect(rect, gradient)

        grid_pen = QtGui.QPen(QtGui.QColor(47, 210, 255, 26), 1)
        painter.setPen(grid_pen)
        for x in range(24, rect.width(), 56):
            painter.drawLine(x, 0, x - 36, rect.height())
        for y in range(28, rect.height(), 48):
            painter.drawLine(0, y, rect.width(), y + 18)

        for index in range(70):
            if rect.width() <= 0 or rect.height() <= 0:
                break
            x = (index * 173 + 31) % rect.width()
            y = (index * 97 + 19) % rect.height()
            color = QtGui.QColor("#34f5ff" if index % 4 else "#d66bff")
            color.setAlpha(42 if index % 3 else 70)
            painter.setPen(QtGui.QPen(color, 1))
            painter.drawPoint(x, y)

        super().paintEvent(event)


class HudFrame(QtWidgets.QFrame):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        rect = self.rect().adjusted(2, 2, -3, -3)
        length = min(34, max(18, rect.width() // 10))
        cyan = QtGui.QPen(QtGui.QColor(62, 246, 255, 190))
        cyan.setWidthF(1.25)
        violet = QtGui.QPen(QtGui.QColor(206, 88, 255, 150))
        violet.setWidthF(1.0)

        painter.setPen(cyan)
        painter.drawLine(rect.topLeft(), rect.topLeft() + QtCore.QPoint(length, 0))
        painter.drawLine(rect.topLeft(), rect.topLeft() + QtCore.QPoint(0, length))
        painter.drawLine(rect.topRight(), rect.topRight() - QtCore.QPoint(length, 0))
        painter.drawLine(rect.topRight(), rect.topRight() + QtCore.QPoint(0, length))
        painter.drawLine(rect.bottomLeft(), rect.bottomLeft() + QtCore.QPoint(length, 0))
        painter.drawLine(rect.bottomLeft(), rect.bottomLeft() - QtCore.QPoint(0, length))
        painter.drawLine(rect.bottomRight(), rect.bottomRight() - QtCore.QPoint(length, 0))
        painter.drawLine(rect.bottomRight(), rect.bottomRight() - QtCore.QPoint(0, length))

        painter.setPen(violet)
        painter.drawLine(rect.left() + length + 8, rect.top(), rect.left() + length + 76, rect.top())


APP_QSS = """
QMainWindow, QWidget#AppRoot {
    background: transparent;
    color: #e9f7ff;
    font-family: "Noto Sans CJK SC", "Microsoft YaHei", "PingFang SC", "Inter", sans-serif;
    font-size: 13px;
}
QFrame#TopBar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 rgba(7, 18, 42, 222),
        stop:0.55 rgba(11, 21, 58, 214),
        stop:1 rgba(32, 9, 48, 206));
    border: 1px solid rgba(71, 220, 255, 130);
    border-radius: 10px;
}
QLabel#AppTitle {
    color: #f4fcff;
    font-size: 22px;
    font-weight: 700;
}
QLabel#AppSubtitle, QLabel#MutedLabel {
    color: #91a9d8;
}
QLabel#StatusChip {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 rgba(13, 35, 75, 220),
        stop:1 rgba(28, 14, 62, 205));
    border: 1px solid rgba(78, 235, 255, 145);
    border-radius: 6px;
    color: #8ff7ff;
    padding: 7px 12px;
    font-weight: 600;
}
QTabWidget::pane {
    border: 1px solid rgba(55, 198, 255, 95);
    border-radius: 10px;
    top: -1px;
    background: rgba(4, 10, 24, 178);
}
QTabBar::tab {
    background: rgba(7, 18, 42, 224);
    color: #9bb4d9;
    border: 1px solid rgba(54, 154, 205, 112);
    border-bottom-color: rgba(54, 139, 189, 70);
    padding: 10px 12px;
    margin-right: 2px;
    min-width: 88px;
    font-weight: 600;
}
QTabBar::tab:selected {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 rgba(7, 40, 74, 236),
        stop:0.55 rgba(18, 31, 77, 232),
        stop:1 rgba(42, 15, 77, 226));
    color: #ffffff;
    border-color: rgba(70, 245, 255, 190);
    border-bottom: 2px solid #42f7ff;
}
QTabBar::tab:hover {
    background: rgba(17, 35, 71, 230);
    color: #d9fbff;
}
QFrame#Panel {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 rgba(7, 23, 49, 236),
        stop:0.55 rgba(8, 16, 42, 228),
        stop:1 rgba(25, 11, 45, 220));
    border: 1px solid rgba(76, 227, 255, 154);
    border-radius: 10px;
}
QFrame#Panel[variant="soft"] {
    background: rgba(9, 18, 43, 205);
}
QLabel#PanelTitle {
    color: #f3fbff;
    font-size: 15px;
    font-weight: 700;
}
QLabel#PanelCaption {
    color: #8499c9;
    font-size: 12px;
}
QLabel#MetricChip {
    background: rgba(4, 11, 27, 218);
    border: 1px solid rgba(73, 158, 218, 104);
    border-radius: 6px;
    color: #d9ecff;
    padding: 7px 9px;
}
QLabel#MetricChip[accent="true"] {
    color: #78fbff;
    border-color: rgba(66, 247, 255, 190);
    background: rgba(5, 39, 61, 230);
}
QLabel#StateBanner {
    background: rgba(5, 12, 29, 224);
    border: 1px solid rgba(77, 176, 244, 120);
    border-left: 4px solid #42f7ff;
    border-radius: 6px;
    color: #eaf9ff;
    padding: 12px 14px;
    font-size: 14px;
    font-weight: 600;
}
QLabel#CameraFrame {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 #02050d,
        stop:0.55 #05091a,
        stop:1 #0a0412);
    border: 1px solid rgba(74, 238, 255, 150);
    border-radius: 10px;
    color: #758ab9;
    font-size: 18px;
}
QTextEdit, QPlainTextEdit {
    background: rgba(3, 8, 21, 230);
    border: 1px solid rgba(70, 143, 205, 112);
    border-radius: 7px;
    color: #e7f6ff;
    selection-background-color: #124f75;
    padding: 8px;
}
QTableWidget {
    background: rgba(3, 8, 21, 230);
    border: 1px solid rgba(70, 143, 205, 112);
    border-radius: 7px;
    color: #e7f6ff;
    gridline-color: rgba(73, 158, 218, 72);
    selection-background-color: #124f75;
    selection-color: #ffffff;
}
QHeaderView::section {
    background: rgba(20, 37, 74, 232);
    color: #a8c9f5;
    border: 0;
    border-right: 1px solid rgba(74, 188, 248, 70);
    border-bottom: 1px solid rgba(74, 188, 248, 70);
    padding: 7px 8px;
    font-weight: 700;
}
QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 rgba(14, 36, 76, 242),
        stop:1 rgba(31, 22, 72, 238));
    border: 1px solid rgba(90, 211, 255, 178);
    border-radius: 7px;
    color: #edf8ff;
    padding: 9px 14px;
    font-weight: 700;
}
QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 rgba(12, 74, 112, 248),
        stop:1 rgba(72, 35, 116, 244));
    border-color: rgba(118, 250, 255, 238);
    color: #ffffff;
}
QPushButton:pressed {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #22d3ee,
        stop:0.58 #38bdf8,
        stop:1 #a855f7);
    border: 2px solid #dffeff;
    color: #ffffff;
    padding: 11px 13px 7px 13px;
}
QPushButton:disabled {
    background: rgba(15, 23, 43, 188);
    border: 1px solid rgba(86, 112, 150, 94);
    color: rgba(145, 164, 195, 128);
}
QPushButton#PrimaryButton {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #0891b2,
        stop:0.56 #12c4dd,
        stop:1 #7c3aed);
    border-color: #64fbff;
    color: #f4feff;
}
QPushButton#PrimaryButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #0ea5c6,
        stop:0.56 #22d3ee,
        stop:1 #9b5cff);
}
QPushButton#PrimaryButton:pressed {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #67f4ff,
        stop:0.52 #36ddff,
        stop:1 #c084fc);
    border: 2px solid #ffffff;
    color: #071426;
    padding: 11px 13px 7px 13px;
}
QPushButton#DangerButton {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #a7193c,
        stop:1 #ff355d);
    border-color: #ff7894;
}
QPushButton#DangerButton:hover {
    background: #ff315c;
}
QPushButton#DangerButton:pressed {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #ff5e7d,
        stop:0.55 #ff3b69,
        stop:1 #ff79b0);
    border: 2px solid #ffe9f0;
    color: #250611;
    padding: 11px 13px 7px 13px;
}
QPushButton#PrimaryButton:disabled, QPushButton#DangerButton:disabled {
    background: rgba(9, 15, 29, 214);
    border: 1px solid rgba(86, 112, 150, 92);
    color: rgba(145, 164, 195, 118);
}
QPushButton#AxisButton {
    min-height: 44px;
    font-size: 15px;
}
QSlider::groove:horizontal {
    height: 7px;
    border-radius: 3px;
    background: rgba(31, 52, 91, 225);
}
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #42f7ff,
        stop:1 #bf5cff);
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #f7fbff;
    border: 2px solid #42f7ff;
    width: 18px;
    height: 18px;
    margin: -7px 0;
    border-radius: 9px;
}
QDoubleSpinBox {
    background: rgba(3, 8, 21, 230);
    border: 1px solid rgba(70, 143, 205, 112);
    border-radius: 7px;
    color: #f7fbff;
    padding: 7px 10px;
}
QSpinBox, QLineEdit, QComboBox {
    background: rgba(3, 8, 21, 230);
    border: 1px solid rgba(70, 143, 205, 112);
    border-radius: 7px;
    color: #f7fbff;
    padding: 7px 10px;
}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {
    border: 1px solid #5cf6ff;
    background: rgba(5, 22, 45, 240);
}
QComboBox::drop-down {
    border: 0;
    width: 24px;
}
QComboBox QAbstractItemView {
    background: #071226;
    border: 1px solid rgba(70, 245, 255, 170);
    color: #e9f7ff;
    selection-background-color: #123f73;
}
QCheckBox {
    color: #e9f7ff;
    spacing: 10px;
    padding: 4px 2px;
}
QCheckBox::indicator {
    width: 17px;
    height: 17px;
    border-radius: 5px;
    border: 1px solid rgba(88, 241, 255, 210);
    background: rgba(3, 8, 21, 230);
}
QCheckBox::indicator:checked {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 #67f4ff,
        stop:0.55 #22d3ee,
        stop:1 #a855f7);
    border: 2px solid #e8feff;
}
QGroupBox {
    background: rgba(6, 14, 34, 206);
    border: 1px solid rgba(73, 188, 238, 122);
    border-radius: 8px;
    color: #dff8ff;
    font-weight: 700;
    margin-top: 13px;
    padding: 16px 10px 10px 10px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 35px;
    padding: 0 8px 0 4px;
    color: #bff8ff;
    background: #09142e;
}
QGroupBox::indicator {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 11px;
    width: 17px;
    height: 17px;
    border-radius: 5px;
    border: 1px solid rgba(88, 241, 255, 210);
    background: rgba(3, 8, 21, 244);
}
QGroupBox::indicator:checked {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 #67f4ff,
        stop:0.55 #22d3ee,
        stop:1 #a855f7);
    border: 2px solid #e8feff;
}
QScrollArea, QScrollArea > QWidget > QWidget {
    background: transparent;
    border: 0;
}
QScrollBar:vertical {
    background: rgba(4, 10, 24, 210);
    width: 10px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: rgba(71, 220, 255, 120);
    border-radius: 5px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}
QScrollBar:horizontal {
    background: rgba(4, 10, 24, 210);
    height: 10px;
    margin: 0;
}
QScrollBar::handle:horizontal {
    background: rgba(71, 220, 255, 120);
    border-radius: 5px;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}
"""


def apply_app_theme(app):
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor("#040711"))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#e9f7ff"))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor("#030815"))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor("#08122b"))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtGui.QColor("#08122b"))
    palette.setColor(QtGui.QPalette.ToolTipText, QtGui.QColor("#ffffff"))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor("#e9f7ff"))
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor("#101f44"))
    palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor("#edf5ff"))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor("#12c4dd"))
    palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor("#ffffff"))
    app.setPalette(palette)
    app.setStyleSheet(APP_QSS)


def make_vertical_scroll_area(widget, object_name):
    scroll = QtWidgets.QScrollArea()
    scroll.setObjectName(str(object_name))
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
    scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
    scroll.setWidget(widget)
    return scroll


def panel(title, caption=None, variant=None):
    frame = HudFrame()
    frame.setObjectName("Panel")
    if variant:
        frame.setProperty("variant", variant)
    layout = QtWidgets.QVBoxLayout(frame)
    layout.setContentsMargins(16, 14, 16, 16)
    layout.setSpacing(12)

    header = QtWidgets.QVBoxLayout()
    header.setSpacing(3)
    title_label = QtWidgets.QLabel(title)
    title_label.setObjectName("PanelTitle")
    header.addWidget(title_label)
    if caption:
        caption_label = QtWidgets.QLabel(caption)
        caption_label.setObjectName("PanelCaption")
        caption_label.setWordWrap(True)
        header.addWidget(caption_label)
    layout.addLayout(header)
    return frame, layout


def metric_chip(text, accent=False):
    chip = QtWidgets.QLabel(text)
    chip.setObjectName("MetricChip")
    chip.setProperty("accent", "true" if accent else "false")
    chip.setAlignment(QtCore.Qt.AlignCenter)
    chip.setMinimumHeight(34)
    return chip


def set_monospace(widget):
    font = QtGui.QFont("JetBrains Mono")
    font.setStyleHint(QtGui.QFont.Monospace)
    font.setPointSize(10)
    widget.setFont(font)
