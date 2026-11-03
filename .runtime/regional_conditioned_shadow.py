"""Bounded previous-read-triggered regional comparison; no production changes."""
import argparse
import atexit
import copy
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import runpy
import sys
import threading

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('conditioned_regional_parent',
                                             HERE/'regional_stationary_shadow.py')
parent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parent)
stage, ab = parent.stage, parent.ab


class _AuditedRegional:
    """Observe every private filter result, including unsampled reads."""
    def __init__(self, target, observer):
        object.__setattr__(self, 'target', target)
        object.__setattr__(self, 'observer', observer)

    def __getattr__(self, name):
        return getattr(self.target, name)

    def __setattr__(self, name, value):
        setattr(self.target, name, value)

    def process(self, frame, color, source, *args, **kwargs):
        result, audit = self.target.process(frame, color, source, *args, **kwargs)
        self.observer._record_shadow_result(audit, source)
        return result, audit

    def reset(self):
        return self.target.reset()


class RegionalConditionedShadow(parent.RegionalShadow):
    BASELINE_SAMPLES = 2
    CONDITIONAL_INTERVAL = .5

    def __init__(self, camera_class, output):
        self.condition_gate = threading.RLock()
        self._regional = None
        self._condition_camera = None
        self.latest_shadow_audit = self.latest_shadow_source = None
        self.previous_condition = None
        self._selection_previous = None
        self._selection_kind = None
        self.condition_counts = dict(reads=0, complete=0, rejected=0,
                                    original_error=0, audit_unavailable=0,
                                    baseline_selected=0, conditional_selected=0)
        self.condition_cross_table = {'complete': {}, 'rejected': {}, 'original_error': {}}
        self.region_reason_counts = {'complete': {}, 'rejected': {}, 'original_error': {}}
        super().__init__(camera_class, output)
        self.baseline_interval = self.interval
        self.original_stable_reads = self.stable_reads

    @property
    def regional(self):
        return self._regional

    @regional.setter
    def regional(self, value):
        self._regional = _AuditedRegional(value, self) if value is not None else None

    def _record_shadow_result(self, audit, source):
        # This is the actual source passed to this read's SDK tap. Published
        # image metadata is assigned later and may still describe an older read.
        self.latest_shadow_audit = copy.deepcopy(audit)
        self.latest_shadow_source = copy.deepcopy(source)

    def _actual_condition(self, complete=False):
        audit = self.latest_shadow_audit
        camera = self._condition_camera
        global_state = (camera.runtime_profile or {}).get('stationary_temporal', {})
        center = next((r for r in (audit or {}).get('regions', [])
                       if r.get('name') == 'center'), None)
        return dict(read_complete=bool(complete), shadow_audit_available=audit is not None,
            global_active=(global_state.get('active') is True) if audit is not None else None,
            center_active=(center.get('qualified') is True and center.get('filter_applied') is True)
                if center is not None else None,
            center_reason=(center.get('visual') or {}).get('reason') if center else None,
            sdk_color_source=copy.deepcopy(self.latest_shadow_source))

    @staticmethod
    def _condition_matches(value):
        return bool(value and value.get('read_complete') is True
                    and value.get('shadow_audit_available') is True
                    and value.get('global_active') is False
                    and value.get('center_active') is True)

    def _selected_read(self, camera, row, args, kwargs):
        row.update(selection_kind=self._selection_kind,
                   previous_condition=copy.deepcopy(self._selection_previous),
                   selection_uses_previous_completed_read_only=True,
                   current_condition_used_to_reject_sample=False)
        key = 'baseline_selected' if self._selection_kind == 'uniform_baseline' else 'conditional_selected'
        self.condition_counts[key] += 1
        complete = False
        try:
            result = super()._selected_read(camera, row, args, kwargs)
            diagnostic = camera.last_read_diagnostics
            complete = (result[0] is not None and result[1] is not None
                        and isinstance(diagnostic, dict) and diagnostic.get('result') == 'complete')
            return result
        finally:
            # This executes before CameraStageObserver finalizes/enqueues row.
            # In particular, never mutate a row already owned by the writer.
            try:
                row['actual_condition'] = self._actual_condition(complete)
                row['shadow_current_sdk_color_source'] = copy.deepcopy(self.latest_shadow_source)
                row['published_ros_image_stamp_available_at_tap'] = False
            except Exception as exc:
                self._error('conditioned_sample_metadata', exc)
                self.finish('conditioned_sample_metadata_error')

    def _record_completed_read(self, outcome):
        actual = self._actual_condition(outcome == 'complete')
        self.condition_counts['reads'] += 1
        self.condition_counts[outcome] += 1
        self.condition_counts['audit_unavailable'] += int(not actual['shadow_audit_available'])
        def label(value):
            return 'true' if value is True else 'false' if value is False else 'unavailable'
        key = 'global_%s_center_%s' % (label(actual['global_active']), label(actual['center_active']))
        table = self.condition_cross_table[outcome]
        table[key] = table.get(key, 0)+1
        reasons = self.region_reason_counts[outcome]
        for region in (self.latest_shadow_audit or {}).get('regions', []):
            name = str(region.get('name', 'unknown'))
            reason = str((region.get('visual') or {}).get('reason', 'unavailable'))
            qualified = region.get('qualified') is True
            bucket = reasons.setdefault(name, {})
            key = 'qualified_%s:%s' % (label(qualified), reason)
            bucket[key] = bucket.get(key, 0)+1
        # A rejected/exception read breaks the triggering chain. No stale
        # qualifying condition survives a missing pair or a failed read.
        self.previous_condition = actual

    def read(self, camera, *args, **kwargs):
        with self.condition_gate:
            if self.closed:
                return self.original(camera, *args, **kwargs)
            self._condition_camera = camera
            self.latest_shadow_audit = self.latest_shadow_source = None
            self._selection_previous = copy.deepcopy(self.previous_condition)
            baseline = self.condition_counts['baseline_selected'] < self.BASELINE_SAMPLES
            self._selection_kind = 'uniform_baseline' if baseline else 'previous_global_false_center_true'
            allowed = baseline or self._condition_matches(self._selection_previous)
            saved_stable, saved_interval = self.stable_reads, self.interval
            # Reuse the frozen parent's deadlines, read taps and queue limits.
            # Its existing selection predicate cannot pass this temporary
            # diagnostic-only gate on a nonqualifying previous read.
            if not allowed:
                self.stable_reads = max(self.stable_reads, self.consecutive_complete+1)
            self.interval = self.baseline_interval if baseline else self.CONDITIONAL_INTERVAL
            outcome = 'original_error'
            try:
                result = super().read(camera, *args, **kwargs)
                diagnostic = camera.last_read_diagnostics
                outcome = ('complete' if result[0] is not None and result[1] is not None
                           and isinstance(diagnostic, dict) and diagnostic.get('result') == 'complete'
                           else 'rejected')
                return result
            finally:
                self.stable_reads, self.interval = saved_stable, saved_interval
                try:
                    self._record_completed_read(outcome)
                except Exception as exc:
                    self.previous_condition = None
                    self._error('conditioned_read_statistics', exc)
                    self.finish('conditioned_read_statistics_error')
                self._condition_camera = None

    def summary(self):
        # The writer may finish as the inherited outer tap exits. Wait until
        # this read's counters are committed before producing its summary.
        with self.condition_gate:
            return dict(super().summary(),
                conditioned_read_counts=copy.deepcopy(self.condition_counts),
                global_center_cross_table=copy.deepcopy(self.condition_cross_table),
                region_qualification_reason_counts=copy.deepcopy(self.region_reason_counts),
                baseline_samples=self.BASELINE_SAMPLES,
                conditional_samples_maximum=self.samples-self.BASELINE_SAMPLES,
                conditional_interval_sec=self.CONDITIONAL_INTERVAL,
                conditioning='previous completed read: global inactive and center active',
                current_condition_never_filters_selected_sample=True,
                no_condition_deadline_behavior='finish with fewer samples; no forced fill',
                source_binding='actual tap SDK color source; no previous publication metadata')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-script', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('ros_args', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or mount not in args.output.resolve().parents:
        raise RuntimeError('SanStand required; no fallback')
    module = importlib.import_module('alicia_flexible_grasp.vision.realsense_manager')
    observer = RegionalConditionedShadow(module.RealSenseManager, args.output)
    modules = {}
    for name in ('realsense_manager', 'regional_stationary_depth', 'visual_stationary'):
        loaded = importlib.import_module('alicia_flexible_grasp.vision.'+name)
        path = Path(loaded.__file__).resolve()
        modules[name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    for name, path in (('conditioned_shadow', Path(__file__).resolve()),
                       ('regional_shadow_parent', HERE/'regional_stationary_shadow.py')):
        modules[name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (args.output/'loaded_modules.json').write_text(json.dumps(modules, indent=2)+'\n')
    atexit.register(observer.shutdown)
    ros_args = args.ros_args[1:] if args.ros_args[:1] == ['--'] else args.ros_args
    sys.argv = [str(args.camera_script)]+ros_args
    runpy.run_path(str(args.camera_script), run_name='__main__')


if __name__ == '__main__':
    main()
