"""Confirm an initial instance before committing the immutable target lock."""
import numpy as np
from alicia_grasp_modes.tabletop import _matching_candidates, _validate_support_plane_update


class InitialAcquisition:
    def __init__(self, config, minimum_points=120, frames=3):
        self.config = config
        self.minimum_points = minimum_points
        self.frames = frames
        self.reset()

    def reset(self):
        self.samples = []

    def observe(self, raw, silhouette, stamp_ns):
        if silhouette.metrics.get('valid_points', 0) < self.minimum_points:
            self.reset()
            return False, 'initial_instance_insufficient_measured_points'
        if self.samples:
            anchor, previous = self.samples[0], self.samples[-1]
            try:
                if stamp_ns <= previous[2]:
                    self.reset()
                    return False, 'initial_instance_nonmonotonic_source'
                if not _matching_candidates([raw], anchor[0], previous[0], self.config):
                    raise ValueError('initial_instance_geometry_changed')
                _validate_support_plane_update(
                    anchor[0].metrics['support_plane_base'], raw.metrics['support_plane_base'],
                    anchor[0].position_base, self.config, anchor[0].support_footprint_base)
                a, b = anchor[1].mask > 0, silhouette.mask > 0
                iou = np.count_nonzero(a & b) / max(1, np.count_nonzero(a | b))
                if iou < .85:
                    raise ValueError('initial_instance_outline_changed')
            except ValueError:
                self.reset()
        self.samples.append((raw, silhouette, int(stamp_ns)))
        return len(self.samples) >= self.frames, 'initial_instance_confirming_%d_of_%d' % (len(self.samples), self.frames)
