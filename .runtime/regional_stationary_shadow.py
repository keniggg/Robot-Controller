"""Bounded same-input regional comparison; production output is never replaced."""
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
import time

HERE = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stage = load('regional_stage_capture', HERE/'camera_stage_observer.py')
ab = load('regional_sdk_copy', HERE/'camera_preset_ab_observer.py')


class RegionalShadow(stage.CameraStageObserver):
    def __init__(self, camera_class, output):
        self.outer_gate = threading.Lock()
        self.current_row = None
        self.last_context_check = None
        self.regional = self.copier = self.evidence = None
        self.shadow_counts = dict(processed=0, global_active=0, regional_active=0,
                                  processing_sec=0., maximum_processing_sec=0.)
        super().__init__(camera_class, output, seconds=30., samples=8, interval=3.,
                         warmup=2., stable_reads=5, maximum_bytes=64*1024*1024)

    def _selected_read(self, camera, row, args, kwargs):
        self.current_row = row
        try:
            return super()._selected_read(camera, row, args, kwargs)
        finally:
            self.current_row = None

    def read(self, camera, *args, **kwargs):
        with self.outer_gate:
            if self.closed:
                return self.original(camera, *args, **kwargs)
            if self.regional is None:
                try:
                    from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth
                    import rospy
                    self.evidence = ab.RosEvidence(rospy)
                    self.copier = ab.PrivateDepthCopy(camera.rs)
                    def factory(*, alpha, delta, persistence):
                        assert (alpha, delta, persistence) == (.4, 100, 0)
                        result = camera.rs.temporal_filter()
                        for key,value in (('filter_smooth_alpha',.4),('filter_smooth_delta',100.),('holes_fill',0.)):
                            result.set_option(getattr(camera.rs.option,key),value)
                        return result
                    self.regional = RegionalStationaryDepth(factory)
                except Exception as exc:
                    self._error("regional_shadow_initialization", exc)
                    self.finish("regional_shadow_initialization_failed")
                    if self.evidence is not None:
                        self.evidence.close()
                    return self.original(camera, *args, **kwargs)
            original = camera._filter_stationary_depth
            had = '_filter_stationary_depth' in camera.__dict__
            saved = camera.__dict__.get('_filter_stationary_depth')
            # Keep superclass cleanup from exposing an in-flight instance tap.
            with self.state_lock:
                self.inflight += 1
            def tapped(frame, diagnostic=None, color=None, source=None):
                actual = original(frame, diagnostic, color=color, source=source)
                if self.closed:
                    return actual
                began = time.monotonic()
                try:
                    aligned = camera.align
                    if isinstance(aligned, stage.TapProxy):
                        aligned = aligned.target
                    assert (aligned is not None and camera.align_depth_to_color
                            and aligned is camera._sdk_color_alignment_owner)
                    assert camera.mode_depth_preset.mode == 'unknown'
                    assert not camera.mode_depth_preset_cfg.get('regional_stationary_enabled', False)
                    if (self.last_context_check is None or self.current_row is not None
                            or began-self.last_context_check >= .5):
                        self.evidence.check_context()
                        self.last_context_check = began
                    guarded = self.evidence.guard.check(self.evidence.now_ns(), freeze=True)
                    visual = (diagnostic or {}).get('stationary_rgb_evidence', {})
                    encoder = bool(guarded and isinstance(visual, dict)
                        and visual.get('reason') in ('stationary', 'warming_up', 'visual_motion',
                            'source_discontinuity', 'insufficient_tracks', 'insufficient_texture',
                            'insufficient_coverage', 'image_shape_changed'))
                    private = self.copier.copy(frame)
                    result, audit = self.regional.process(private, color, source,
                        camera._diagnostic_clock(), encoder, aligned_to_color=True)
                    elapsed = time.monotonic()-began
                    self.shadow_counts['processed'] += 1
                    self.shadow_counts['global_active'] += int(bool(camera.runtime_profile.get('stationary_temporal',{}).get('active')))
                    self.shadow_counts['regional_active'] += int(bool(audit.get('active')))
                    self.shadow_counts['processing_sec'] += elapsed
                    self.shadow_counts['maximum_processing_sec'] = max(self.shadow_counts['maximum_processing_sec'], elapsed)
                    row = self.current_row
                    if row is not None:
                        self._array(row, 'regional_shadow_depth', result)
                        clipped = camera._clip_depth_range(result)
                        rectified = camera.pair_rectifier.apply(color, clipped)[1] if camera.pair_rectifier else clipped
                        self._array(row, 'regional_shadow_final_depth', rectified)
                        row.update(regional_shadow=copy.deepcopy(audit),
                            shadow_counts=copy.deepcopy(self.shadow_counts),
                            shadow_geometry_authority=False, production_output_replaced=False,
                            shadow_tap_processing_sec=elapsed,
                            feedback_guard=self.evidence.guard.snapshot())
                except Exception as exc:
                    self.regional.reset()
                    self._error('regional_shadow', exc)
                    self.finish('regional_shadow_error')
                return actual
            camera._filter_stationary_depth = tapped
            try:
                result = super().read(camera, *args, **kwargs)
                if (camera.last_read_diagnostics or {}).get('result') != 'complete':
                    self.regional.reset()
                return result
            except BaseException:
                self.regional.reset()
                raise
            finally:
                if camera.__dict__.get('_filter_stationary_depth') is tapped:
                    if had: camera._filter_stationary_depth = saved
                    else: del camera._filter_stationary_depth
                with self.state_lock:
                    self.inflight -= 1
                    if self.closed: self.finish(self.close_reason)

    def _write_row(self, row):
        row['array_coordinate_spaces'].update(
            regional_shadow_depth='SDK depth aligned to color; before range clip and pair rectification',
            regional_shadow_final_depth='published paired rectified geometry; same clip/map as production',
            final_depth='published paired rectified geometry',
            final_rgb='published paired rectified geometry')
        row['source_evidence'] = self.evidence.at_anchor(row['read_completed_wall_ns'])
        return super()._write_row(row)

    def summary(self):
        return dict(super().summary(), regional_shadow_counts=self.shadow_counts,
            geometry_authority=False, production_output_replaced=False,
            context_poll_interval_sec=.5, context_changes_between_polls_not_latched=True,
            temperature_queries_sent=False, motion_commands_sent=False)

    def _writer(self):
        try:
            return super()._writer()
        finally:
            if self.evidence is not None:
                self.evidence.close()

    def shutdown(self):
        super().shutdown()
        if self.evidence is not None:
            self.evidence.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-script',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('ros_args',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    mount=Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or mount not in args.output.resolve().parents:
        raise RuntimeError('SanStand required; no fallback')
    module=importlib.import_module('alicia_flexible_grasp.vision.realsense_manager')
    observer=RegionalShadow(module.RealSenseManager,args.output)
    modules = {}
    for name in ('realsense_manager','regional_stationary_depth','visual_stationary'):
        loaded = importlib.import_module('alicia_flexible_grasp.vision.'+name)
        path = Path(loaded.__file__).resolve()
        modules[name] = dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (args.output/'loaded_modules.json').write_text(json.dumps(modules,indent=2)+'\n')
    atexit.register(observer.shutdown)
    ros_args=args.ros_args[1:] if args.ros_args[:1]==['--'] else args.ros_args
    sys.argv=[str(args.camera_script)]+ros_args
    runpy.run_path(str(args.camera_script),run_name='__main__')


if __name__=='__main__':
    main()
