"""Independent static RGB regions authorize only their own depth history.

Inputs must be an accepted SDK pair with depth already aligned to this color
image. The caller resets this object on every rejected/missing pair. This class
never aligns images, fills depth, establishes target identity or authorizes
motion. The filter factory constructs independent SDK temporal filters.
"""
import math
import numbers

import numpy as np

from .visual_stationary import VisualStationaryWindow


class RegionalStationaryDepth:
    OWNER_SCHEMA = 'fixed_center_02_08_01_09_then_four_quadrants_v1'
    ALPHA = .4
    DELTA = 100
    PERSISTENCE = 0

    def __init__(self, filter_factory):
        if not callable(filter_factory):
            raise ValueError('regional_filter_factory_required')
        self._factory = filter_factory
        self._shape = None
        self._profile = None
        self._previous_source = None
        self._regions = []
        self._nested_shadow_visual = None
        self._nested_shadow_bounds = None

    def reset(self):
        self._previous_source = None
        # This observer has no filter or pixel ownership. Discard its private
        # history whenever production history loses source/geometry permission.
        self._nested_shadow_visual = None
        self._nested_shadow_bounds = None
        for region in self._regions:
            region['visual'].reset()
            region['filter'] = None
            region['anchor'] = None

    def _geometry(self, shape):
        if shape == self._shape:
            return False
        self.reset()
        self._shape = shape
        self._profile = None
        height, width = shape
        center = (int(.2*width), int(.1*height), int(.8*width), int(.9*height))
        boxes = [('center', center),
                 ('top_left', (0, 0, width//2, height//2)),
                 ('top_right', (width//2, 0, width, height//2)),
                 ('bottom_left', (0, height//2, width//2, height)),
                 ('bottom_right', (width//2, height//2, width, height))]
        owner = np.full(shape, 255, np.uint8)
        for index, (_, (x0, y0, x1, y1)) in enumerate(boxes):
            patch = owner[y0:y1, x0:x1]
            patch[patch == 255] = index
        if np.any(owner == 255):
            raise ValueError('regional_owner_geometry_incomplete')
        self._regions = []
        for index, (name, bounds) in enumerate(boxes):
            x0, y0, x1, y1 = bounds
            self._regions.append(dict(name=name, bounds=bounds,
                owned=owner[y0:y1, x0:x1] == index,
                visual=VisualStationaryWindow(), filter=None, anchor=None))
        return True

    @staticmethod
    def _profile_signature(frame):
        profile = frame.get_profile()
        signature = tuple(str(getattr(profile, name)()) for name in
            ('stream_type', 'stream_index', 'format', 'unique_id', 'fps'))
        intrinsic = profile.as_video_stream_profile().get_intrinsics()
        numbers_ = [intrinsic.width, intrinsic.height, intrinsic.fx, intrinsic.fy,
                    intrinsic.ppx, intrinsic.ppy, *intrinsic.coeffs]
        if (not np.all(np.isfinite(numbers_)) or min(numbers_[:4]) <= 0):
            raise ValueError('regional_profile_intrinsics_invalid')
        return signature + tuple(float(v) for v in numbers_) + (str(intrinsic.model),)

    @staticmethod
    def _source_key(source, host):
        if not isinstance(source, dict):
            raise ValueError('regional_source_invalid')
        number, stamp, fps = (source.get(k) for k in
                              ('frame_number', 'device_timestamp_ms', 'stream_fps'))
        domain = str(source.get('timestamp_domain', '')).rsplit('.', 1)[-1]
        if (isinstance(number, bool) or not isinstance(number, numbers.Integral)
                or number < 0 or domain not in ('hardware_clock', 'global_time')):
            raise ValueError('regional_source_invalid')
        for value in (stamp, fps, host):
            if (isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real)
                    or not math.isfinite(value)):
                raise ValueError('regional_source_invalid')
        if stamp < 0 or not 1 <= fps <= 240 or host < 0:
            raise ValueError('regional_source_invalid')
        return int(number), float(stamp), domain, float(fps), float(host)

    def _nested_center_bounds(self):
        if not self._regions:
            return None
        x0, y0, x1, y1 = self._regions[0]['bounds']
        width, height = (x1-x0)//2, (y1-y0)//2
        left, top = x0+(x1-x0-width)//2, y0+(y1-y0-height)//2
        return left, top, left+width, top+height

    def _nested_shadow_report(self, reason):
        bounds = self._nested_center_bounds()
        return dict(diagnostic_only=True, pixels_changed=False,
            used_for_depth_permission=False, motion_authorized=False,
            temporal_filter_created=False,
            bounds_xyxy=None if bounds is None else list(bounds),
            coordinate_domain='sdk_color_aligned_before_pair_rectification',
            reason=reason, qualified=False, visual=None)

    def _observe_nested_center_shadow(self, color, source, host):
        """Observe a smaller fixed crop without affecting the five owners.

        Only an explicitly enabled, finite diagnostic wrapper may call this
        after production processing. Normal process() never computes it.
        Neither its result nor its private state supplies production permission.
        """
        report = self._nested_shadow_report('not_observed')
        try:
            bounds = self._nested_center_bounds()
            if bounds is None:
                return report
            if self._nested_shadow_visual is None or bounds != self._nested_shadow_bounds:
                self._nested_shadow_visual = VisualStationaryWindow()
                self._nested_shadow_bounds = bounds
            x0, y0, x1, y1 = bounds
            # Detached pixels protect the real SDK color even if a diagnostic
            # implementation fails. No depth pixels or temporal filter enter.
            crop = color[y0:y1, x0:x1].copy(order='C')
            visual = self._nested_shadow_visual.observe(crop, dict(source), host)
            report.update(reason=visual.get('reason', 'invalid_evidence'),
                qualified=visual.get('qualified') is True, visual=visual)
        except Exception as exc:
            self._nested_shadow_visual = None
            self._nested_shadow_bounds = None
            report.update(reason='shadow_observation_error', qualified=False,
                          visual=None, error_kind=type(exc).__name__)
        return report

    def _report(self, reason):
        return dict(enabled=True, active=False, fully_active=False,
            whole_frame_stationarity_qualified=False,
            owner_schema=self.OWNER_SCHEMA, reason=reason,
            alpha=self.ALPHA, delta_depth_units=self.DELTA, persistence=self.PERSISTENCE,
            coordinate_domain='sdk_color_aligned_before_pair_rectification',
            filtered_pixel_count=0, filtered_fraction=0., changed_pixel_count=0,
            current_zero_pixels_preserved=True, regions=[],
            nested_center_shadow=self._nested_shadow_report('diagnostic_disabled'))

    def process(self, frame, color, source, host, encoder_active,
                aligned_to_color=False):
        """Return a detached uint16 array and JSON diagnostic, never an SDK frame.

        ``filter_factory(alpha=.4, delta=100, persistence=0)`` must return a
        NEW configured filter each time. Every region sees the same complete
        pre-stationary SDK frame, never another region's filtered output.
        """
        try:
            depth = np.asanyarray(frame.get_data())
            if depth.dtype != np.uint16 or depth.ndim != 2 or not depth.size:
                raise ValueError('regional_depth_shape_or_type_invalid')
            current = depth.copy()
            geometry_changed = self._geometry(current.shape)
            report = self._report('regional_observation')
            if aligned_to_color is not True or encoder_active is not True:
                self.reset()
                report['reason'] = ('sdk_color_alignment_unproven' if aligned_to_color is not True
                                    else 'encoder_or_mode_inactive')
                return current, report
            if (not isinstance(color, np.ndarray) or color.dtype != np.uint8
                    or color.shape != current.shape+(3,)):
                self.reset()
                report['reason'] = 'current_color_geometry_invalid'
                return current, report
            try:
                key = self._source_key(source, host)
                profile = self._profile_signature(frame)
                if profile[5:7] != (current.shape[1], current.shape[0]):
                    raise ValueError('regional_profile_image_shape_mismatch')
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.reset()
                report['reason'] = 'source_or_frame_profile_invalid'
                return current, report
            global_reset = 'image_geometry_changed' if geometry_changed else None
            if self._profile is not None and profile != self._profile:
                self.reset()
                global_reset = 'sdk_profile_changed'
            self._profile = profile
            if self._previous_source is not None:
                previous = self._previous_source
                dt = (key[1]-previous[1])/1000.
                host_dt = key[4]-previous[4]
                if (key[0] != previous[0]+1 or key[2:4] != previous[2:4]
                        or not .25/key[3] <= dt <= 1.75/key[3]
                        or not 0 < host_dt <= max(.1, 2.5/key[3])):
                    self.reset()
                    global_reset = 'sdk_source_discontinuity'
            self._previous_source = key
            report['global_history_reset_reason'] = global_reset
            output = current.copy()
            zero = current == 0
            qualified_pixels = 0
            for region in self._regions:
                x0, y0, x1, y1 = region['bounds']
                crop = np.ascontiguousarray(color[y0:y1, x0:x1])
                visual = region['visual'].observe(crop, source, host)
                anchor = region['visual']._anchor
                active = visual.get('qualified') is True and anchor is not None
                warming = visual.get('reason') == 'warming_up' and anchor is not None
                anchor_changed = region['anchor'] is not anchor
                reset = anchor_changed or not (active or warming)
                if reset:
                    region['filter'] = None
                    region['anchor'] = None
                applied = False
                if active or warming:
                    region['anchor'] = anchor
                    if region['filter'] is None:
                        created = self._factory(alpha=self.ALPHA, delta=self.DELTA,
                                                persistence=self.PERSISTENCE)
                        if (created is None or any(created is r['filter'] for r in self._regions)):
                            raise ValueError('regional_filter_factory_must_return_independent_filters')
                        region['filter'] = created
                    result = region['filter'].process(frame)
                    filtered = np.asanyarray(result.get_data())
                    if filtered.shape != current.shape or filtered.dtype != current.dtype:
                        raise ValueError('regional_temporal_output_geometry_changed')
                    if np.any(filtered[zero] != 0):
                        raise ValueError('regional_temporal_filled_current_depth_holes')
                    if not np.array_equal(np.asanyarray(frame.get_data()), current):
                        raise ValueError('regional_temporal_mutated_current_input')
                    if active:
                        patch = output[y0:y1, x0:x1]
                        patch[region['owned']] = filtered[y0:y1, x0:x1][region['owned']]
                        qualified_pixels += int(np.count_nonzero(region['owned']))
                        applied = True
                    # No SDK frame handle is retained beyond this iteration.
                    del result, filtered
                report['regions'].append(dict(name=region['name'],
                    bounds_xyxy=list(region['bounds']), owned_pixels=int(np.count_nonzero(region['owned'])),
                    qualified=active, warming=bool(warming and not active), history_reset=bool(reset),
                    filter_applied=applied, visual=visual))
            report.update(active=bool(qualified_pixels), fully_active=qualified_pixels == current.size,
                filtered_pixel_count=qualified_pixels, filtered_fraction=qualified_pixels/float(current.size),
                changed_pixel_count=int(np.count_nonzero(output != current)))
            return output, report
        except Exception:
            self.reset()
            raise
