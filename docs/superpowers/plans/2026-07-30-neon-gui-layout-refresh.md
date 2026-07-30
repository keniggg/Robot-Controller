# Neon GUI Layout Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the Alicia-D cyan/violet neon HUD while making button
hierarchy and pressed feedback obvious and eliminating visible control overlap
at `1120 x 720`.

**Architecture:** Centralize visual behavior and scroll-container construction
in `gui/theme.py`. Keep existing widgets, signals, ROS interfaces, and tab
ordering intact; change only their layout containment. The overview right stack
and target-recognition controls receive independent vertical scroll areas, and
the compact 6D action row becomes a two-column grid.

**Tech Stack:** Python 3, ROS Noetic, PyQt5/Fusion style, Qt Style Sheets,
`unittest`, catkin-generated Python messages/services, X11 screenshot probes.

## Global Constraints

- Preserve all ROS topics, services, parameters, callbacks, timers, button
  labels, button signal connections, and enabled-state rules.
- Do not start the arm driver, motion gateway, camera, tactile hardware, or any
  other hardware node during GUI verification.
- Do not publish motion, enable, disable, stop, torque-off, controller-stop, or
  emergency commands.
- Keep the eight tabs in their current order.
- Keep the window minimum size at exactly `1120 x 720`.
- Button pressed feedback is momentary only: brighter background, stronger
  border, and approximately two-pixel label displacement.
- Verify both `1120 x 720` and `1320 x 860`.

---

### Task 1: Lock the visual and layout contracts with failing tests

**Files:**

- Create:
  `src/alicia_flexible_grasp_supervisor/tests/test_gui_layout_contract.py`
- Read:
  `src/alicia_flexible_grasp_supervisor/gui/theme.py`
- Read:
  `src/alicia_flexible_grasp_supervisor/gui/main_gui.py`
- Read:
  `src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py`
- Read:
  `src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py`

**Interfaces:**

- Consumes: real `MainWindow`, `PerceptionWidget`, and
  `Grasp6DControlWidget` Qt components with only external ROS endpoints
  replaced by inert test doubles.
- Produces: behavioral tests for
  `make_vertical_scroll_area(widget, object_name)`,
  `OverviewStatusScroll`, `PerceptionControlsScroll`, compact action-grid
  layout, pressed selectors, disabled selectors, and group-box indicators.

- [ ] **Step 1: Add real Qt interaction-state tests**

Create an offscreen `QApplication`, apply the production theme, and render a
real `PrimaryButton` before and during `QTest.mousePress`. Assert the two images
differ, the pressed center pixel is brighter, and the styled content rectangle
moves downward. Release the mouse and assert the original render returns.

```python
QtTest.QTest.mousePress(button, QtCore.Qt.LeftButton)
app.processEvents()
pressed = button.grab().toImage()
assert pressed != normal
assert pressed_content.top() > normal_content.top()
```

Render a real checkable `QGroupBox` and use
`QStyle.subControlRect(QStyle.CC_GroupBox, ...)` to assert its indicator is at
least `17 x 17` and does not intersect its title rectangle.

- [ ] **Step 2: Add scroll-helper behavior tests**

Create a `QApplication` with `QT_QPA_PLATFORM=offscreen`, pass a plain child
widget to `make_vertical_scroll_area`, and assert:

```python
scroll = make_vertical_scroll_area(child, 'ProbeScroll')
assert scroll.objectName() == 'ProbeScroll'
assert scroll.widget() is child
assert scroll.widgetResizable()
assert scroll.horizontalScrollBarPolicy() == QtCore.Qt.ScrollBarAlwaysOff
assert scroll.verticalScrollBarPolicy() == QtCore.Qt.ScrollBarAsNeeded
assert scroll.frameShape() == QtWidgets.QFrame.NoFrame
```

- [ ] **Step 3: Add real component containment tests**

Patch only `rospy.Publisher`, `rospy.Subscriber`, and `rospy.get_param`, then
instantiate the real `MainWindow` at `1120 x 720`. Assert:

```python
overview = window.findChild(QtWidgets.QScrollArea, 'OverviewStatusScroll')
perception = window.findChild(
    QtWidgets.QScrollArea,
    'PerceptionControlsScroll',
)
assert overview is not None
assert perception is not None
assert window.findChild(QtWidgets.QTabWidget).count() == 8
```

Find the compact `Grasp6DControlWidget` and its named `CompactActionGrid`.
Assert the four existing button objects occupy four distinct grid cells and
that all button labels remain fully contained by their rectangles after
layout. Select every tab and assert every tab rectangle fits inside the tab
bar without visible scroll buttons.

```python
assert grid.itemAtPosition(0, 0).widget() is compact.check_btn
assert grid.itemAtPosition(0, 1).widget() is compact.request_plan_btn
assert grid.itemAtPosition(1, 0).widget() is compact.execute_btn
assert grid.itemAtPosition(1, 1).widget() is compact.stop_btn
```

- [ ] **Step 4: Run the focused test and record RED**

Run:

```bash
source devel/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_gui_layout_contract -v
```

Expected: failure because `make_vertical_scroll_area`,
`OverviewStatusScroll`, `PerceptionControlsScroll`, compact grid branching, and
the new style selectors do not yet exist.

---

### Task 2: Refine the shared neon theme and scroll primitive

**Files:**

- Modify: `src/alicia_flexible_grasp_supervisor/gui/theme.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_gui_layout_contract.py`

**Interfaces:**

- Consumes: `PyQt5.QtCore`, `PyQt5.QtGui`, `PyQt5.QtWidgets`.
- Produces:
  `make_vertical_scroll_area(widget: QWidget, object_name: str) -> QScrollArea`
  and the refined `APP_QSS`.

- [ ] **Step 1: Add the vertical scroll-area factory**

Implement:

```python
def make_vertical_scroll_area(widget, object_name):
    scroll = QtWidgets.QScrollArea()
    scroll.setObjectName(str(object_name))
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
    scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
    scroll.setWidget(widget)
    return scroll
```

The helper must not alter the child widget or its ROS behavior.

- [ ] **Step 2: Refine tabs, panels, inputs, and scroll bars**

Keep current colors but:

- change tab padding to `10px 12px`, margin-right to `2px`, and minimum width
  to `88px`;
- strengthen panel fill contrast while retaining cyan/violet borders;
- style `QScrollArea`, `QScrollArea > QWidget > QWidget`, and both scrollbar
  orientations with transparent containers and restrained neon handles;
- keep the existing font family and base size.

- [ ] **Step 3: Add complete button interaction states**

Use explicit normal, hover, pressed, and disabled states. The general pressed
selector must include:

```css
QPushButton:pressed {
    border: 2px solid #dffeff;
    padding: 11px 14px 7px 14px;
}
```

Add brighter pressed gradients for `PrimaryButton` and `DangerButton`. Do not
make any button checkable.

- [ ] **Step 4: Add checkbox and group-box indicators**

Define separate blocks for:

```css
QCheckBox::indicator
QCheckBox::indicator:checked
QGroupBox
QGroupBox::title
QGroupBox::indicator
QGroupBox::indicator:checked
```

Use fixed `17px` indicators and group-title padding that reserves indicator
space. Preserve the existing `QGroupBox.toggled` behavior.

- [ ] **Step 5: Run Task 1 theme/helper tests**

Run the focused command from Task 1.

Expected: theme/helper behavior passes; containment behavior remains RED until
Tasks 3 and 4.

---

### Task 3: Make overview and compact 6D controls responsive

**Files:**

- Modify: `src/alicia_flexible_grasp_supervisor/gui/main_gui.py`
- Modify:
  `src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_gui_layout_contract.py`

**Interfaces:**

- Consumes: `make_vertical_scroll_area(widget, object_name)` from Task 2.
- Produces: `OverviewStatusScroll` and the existing four compact 6D actions in
  a two-column grid.

- [ ] **Step 1: Wrap the overview status stack**

Replace the bare right-side `QVBoxLayout` with a content widget:

```python
right_content = QtWidgets.QWidget()
right = QtWidgets.QVBoxLayout(right_content)
right.setContentsMargins(0, 0, 0, 0)
right.setSpacing(14)
```

Add the same robot, tactile, and grasp widgets in the same order, add a stretch,
then add:

```python
h.addWidget(
    make_vertical_scroll_area(right_content, 'OverviewStatusScroll'),
    4,
)
```

Keep the camera at stretch factor `7`.

- [ ] **Step 2: Use a compact-only action grid**

In `Grasp6DControlWidget`, retain all button construction and signal
connections. Use:

```python
if self._compact:
    actions = QtWidgets.QGridLayout()
    actions.addWidget(self.check_btn, 0, 0)
    actions.addWidget(self.request_plan_btn, 0, 1)
    actions.addWidget(self.execute_btn, 1, 0)
    actions.addWidget(self.stop_btn, 1, 1)
else:
    actions = QtWidgets.QHBoxLayout()
```

Non-compact layout remains one row with the existing stretch factors.

- [ ] **Step 3: Run the focused GUI contract test**

Run the Task 1 command.

Expected: all assertions except `PerceptionControlsScroll` pass.

---

### Task 4: Make target-recognition controls independently scrollable

**Files:**

- Modify:
  `src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_gui_layout_contract.py`

**Interfaces:**

- Consumes: `make_vertical_scroll_area(widget, object_name)` from Task 2.
- Produces: `PerceptionControlsScroll`, containing the unchanged left control
  panel.

- [ ] **Step 1: Convert the left controls into a content widget**

Replace the bare `controls` layout with:

```python
controls_content = QtWidgets.QWidget()
controls = QtWidgets.QVBoxLayout(controls_content)
controls.setContentsMargins(0, 0, 0, 0)
controls.setSpacing(0)
```

Keep every existing child and connection in its current order.

- [ ] **Step 2: Insert the control scroll area**

Replace `layout.addLayout(controls, 4)` with:

```python
layout.addWidget(
    make_vertical_scroll_area(
        controls_content,
        'PerceptionControlsScroll',
    ),
    4,
)
```

Keep the camera preview at stretch factor `3`.

- [ ] **Step 3: Run the complete focused GUI contract**

Run the Task 1 command.

Expected: PASS.

- [ ] **Step 4: Run existing GUI regressions**

Run:

```bash
source devel/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_gui_layout_contract \
  src.alicia_flexible_grasp_supervisor.tests.test_gui_ros_lifecycle \
  src.alicia_flexible_grasp_supervisor.tests.test_gui_grasp_mode_helpers \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp6d_control_widget \
  src.alicia_flexible_grasp_supervisor.tests.test_perception_model_selection_widget \
  src.alicia_flexible_grasp_supervisor.tests.test_tcp_calibration_widget -q
```

Expected: all tests pass.

---

### Task 5: Render, verify, document, commit, and push

**Files:**

- Modify:
  `src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`
- Verify all production/test files from Tasks 1–4.

**Interfaces:**

- Consumes: completed responsive GUI and current `gui.launch`.
- Produces: layout screenshots, runtime-log evidence, committed and pushed
  branch.

- [ ] **Step 1: Compile and run static checks**

Run:

```bash
source devel/setup.bash
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/gui/theme.py \
  src/alicia_flexible_grasp_supervisor/gui/main_gui.py \
  src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py \
  src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py
git diff --check
```

Expected: both commands exit `0`.

- [ ] **Step 2: Capture every tab at both supported sizes**

With only `roscore` and `gui.launch` running, instantiate the latest
`MainWindow`, select each tab, process Qt events, and save window grabs at
`1120 x 720` and `1320 x 860` under `/tmp`.

Record each tab's text and size hint. Confirm:

- no visible control rectangles overlap;
- the overview right stack scrolls instead of compressing;
- target-recognition controls scroll independently;
- all eight tabs fit without navigation arrows;
- all button labels remain readable.

- [ ] **Step 3: Capture pressed-state evidence**

Use Qt mouse events on a non-hardware probe button styled as
`PrimaryButton`. Capture normal and pressed images without clicking any real
GUI action. Confirm the pressed image has brighter fill/border and the label is
visibly lower.

- [ ] **Step 4: Run the full supervisor regression**

Run:

```bash
source devel/setup.bash
python3 -m unittest discover \
  -s src/alicia_flexible_grasp_supervisor/tests -q
```

Expected: all tests pass.

- [ ] **Step 5: Append runtime evidence**

Append a dated entry recording:

- only `roscore` and `gui.launch` were used;
- no hardware nodes or hardware commands ran;
- the original `1239 x 916` overview and `999 x 1100` perception size hints;
- the root cause and layout changes;
- focused/full test counts and screenshot sizes;
- ROS interfaces and functions remained unchanged.

- [ ] **Step 6: Commit and push**

Stage only the design plan, GUI production files, layout test, and runtime log.
Commit with:

```bash
git commit -m "style: refine neon ROS GUI layout"
git push origin codex/protocol-v3-upgrade
```

Verify local `HEAD` equals `origin/codex/protocol-v3-upgrade` and the worktree
is clean.
