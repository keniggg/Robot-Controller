# Neon GUI Layout Refresh Design

**Date:** 2026-07-30

**Status:** Approved for implementation

## Goal

Refresh the existing Alicia-D ROS GUI without changing its functions, ROS
topics, services, button connections, or hardware behavior. Preserve the
current cyan/violet neon HUD identity, make action hierarchy clearer, add
strong momentary button feedback, and eliminate control overlap at the
supported minimum window size of `1120 x 720`.

## Observed Problem

The current GUI was rendered from the latest
`codex/protocol-v3-upgrade` worktree with only `roscore` and `gui.launch`;
no hardware node was started.

At `1120 x 720`:

- the overview tab reports a `1239 x 916` size hint, so the right-hand robot,
  tactile, and compact 6D panels are compressed until text and buttons overlap;
- the target-recognition tab reports a `999 x 1100` size hint, so the visual
  alignment controls, metrics, action note, and buttons overlap vertically;
- the checkable `QGroupBox` has no dedicated indicator/title styling, leaving
  the advanced-parameter selector visually inconsistent with the neon theme;
- the eight tabs exceed the available tab-bar width and unnecessarily fall
  back to navigation arrows;
- buttons use color variants, but the pressed state is not sufficiently
  distinct from hover.

The overlap is caused by large fixed/minimum child sizes being forced into
non-scrolling tab layouts whose available size is smaller than their size
hint. It is not caused by ROS data or button behavior.

## Selected Direction

Use a systematic refinement of the existing neon HUD:

- keep the dark navy background, cyan focus color, violet secondary accent,
  and magenta/red destructive-action color;
- reduce decorative noise slightly while improving contrast and spacing;
- use layout containment and scrolling instead of shrinking controls below
  readable dimensions;
- keep all user-visible actions and their existing meaning unchanged.

## Visual System

### Panels and Typography

- Retain the gradient application background and HUD corner marks.
- Use a consistent panel fill, one-pixel cyan border, and restrained violet
  accent.
- Keep Chinese UI text at the existing readable base size.
- Improve muted text contrast without competing with active values.
- Use consistent internal spacing so adjacent status chips do not touch.

### Button Hierarchy

- Primary actions use a bright cyan-to-violet gradient and stronger cyan
  border.
- Destructive or execution actions retain a magenta/red treatment.
- Secondary actions use a darker surface with a clear cyan outline.
- Disabled buttons remain visibly disabled and cannot resemble active
  controls.

All buttons receive the same momentary pressed feedback:

1. the background becomes visibly brighter;
2. the border becomes brighter and thicker-looking through higher contrast;
3. top/bottom padding shifts the label downward by approximately two pixels;
4. releasing the mouse restores the normal state.

No button becomes checkable and no persistent state is added.

### Check Boxes and Checkable Groups

- Style `QCheckBox::indicator` and `QGroupBox::indicator` separately.
- Give each indicator a fixed hit-area-compatible size and explicit spacing.
- Reserve title space in `QGroupBox` so its indicator cannot overlap the title.
- Use cyan fill and a visible check mark or contrasting center when checked.
- Preserve all current checked defaults and signals.

## Layout Changes

### Main Shell and Tabs

- Keep the top bar and all eight existing tabs in the same order.
- Reduce tab minimum width and horizontal padding enough for the complete tab
  row to fit at `1120 x 720`.
- Do not rename, remove, or reorder tabs.

### Overview Tab

- Keep the camera on the left and status panels on the right.
- Place the right-hand robot/tactile/compact-6D stack in a borderless vertical
  scroll area. Scrolling appears only when required.
- Change compact 6D actions from one four-button row to a `2 x 2` grid so
  labels remain readable at the minimum width.
- Keep the camera viewport and all ROS subscriptions unchanged.

### Target-Recognition Tab

- Keep controls on the left and the RGB-D preview on the right.
- Place the left control panel in a borderless vertical scroll area so the
  alignment group, metrics, mode note, actions, and status retain their normal
  height.
- Keep advanced parameters collapsed by default and preserve the current
  checkable group behavior.
- Keep all model selection, recognition, alignment, planning, and grasp signal
  connections unchanged.

### Remaining Tabs

- Preserve their current component structure.
- Apply shared theme improvements and spacing only.
- Do not introduce scrolling unless the layout probe proves a minimum-size
  overlap after the shared style update.

## Functional Invariants

The implementation must not change:

- ROS node names, topics, services, message types, or parameters;
- publishers, subscribers, timers, callbacks, or service invocation code;
- button labels, signal connections, enabled-state rules, or confirmation
  dialogs;
- joint direct-control semantics, planning/execution separation, grasp
  authority, or stop behavior;
- camera processing, target recognition, calibration, tactile, or log data
  flow;
- automatic or manual hardware command behavior.

Opening the GUI for visual verification must use only `roscore` and
`gui.launch`; it must not start the arm driver or other hardware nodes.

## Implementation Boundaries

Expected production changes are limited to:

- `src/alicia_flexible_grasp_supervisor/gui/theme.py`
- `src/alicia_flexible_grasp_supervisor/gui/main_gui.py`
- `src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py`
- layout-focused GUI tests

`perception_widget.py` should not need functional edits because its entire
existing left-side widget can be placed in a scroll container by the main
shell. If investigation shows the container must be created inside that
widget, the change must remain layout-only.

## Verification

Automated verification will cover:

- theme selectors for normal, hover, pressed, disabled, checkbox, and
  checkable-group states;
- unchanged button labels and signal connections;
- scroll-area containment for the overview status stack and target controls;
- compact 6D action layout with all four existing actions;
- layout probes at `1120 x 720` and `1320 x 860`;
- existing GUI widget, ROS lifecycle, grasp-mode, perception-model, TCP
  calibration, and 6D control tests;
- Python compilation and `git diff --check`.

Visual verification will capture every tab at both supported sizes. Completion
requires no overlapping visible child controls, readable button labels, a
complete tab row, and clearly distinct normal, hover, and pressed button
states.

