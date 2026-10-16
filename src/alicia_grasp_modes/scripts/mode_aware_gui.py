#!/usr/bin/env python3
"""Embed the grasp selectors without modifying the original GUI source."""
import importlib.util
from pathlib import Path
import sys

import rospkg

root = Path(rospkg.RosPack().get_path('alicia_flexible_grasp_supervisor'))
sys.path.insert(0, str(root))
spec = importlib.util.spec_from_file_location('_alicia_original_gui', root / 'gui' / 'main_gui.py')
original = importlib.util.module_from_spec(spec)
spec.loader.exec_module(original)

from alicia_grasp_modes.mode_widget import GraspModeWidget


class ModeAwareGraspControl(original.Grasp6DControlWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.mode_widget = GraspModeWidget(self)
        self.layout().insertWidget(0, self.mode_widget)

    def _shutdown_ros(self):
        widget = self.__dict__.get('mode_widget')
        if widget is not None:
            widget._shutdown_ros()
        super()._shutdown_ros()


original.Grasp6DControlWidget = ModeAwareGraspControl

if __name__ == '__main__':
    original.main()
