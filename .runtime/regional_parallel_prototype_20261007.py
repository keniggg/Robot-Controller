"""OFFLINE ONLY three-worker prototype; never imported by production entrypoints.

The original process prefix/suffix is reused from its inspected source. Only
the independent five-region loop is replaced. All jobs finish before ordered
merge or exception/reset; factories are serialized and checked for sharing.
"""
import inspect
import textwrap
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait

import numpy as np

from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth


class ParallelRegionalPrototype(RegionalStationaryDepth):
    def __init__(self, factory, workers=3):
        if type(workers) is not int or not 1 <= workers <= 3:
            raise ValueError('offline_worker_bound')
        super().__init__(factory)
        self._offline_pool = ThreadPoolExecutor(max_workers=workers,
                                                thread_name_prefix='offline_region')
        self._offline_factory_lock = threading.Lock()
        self.last_jobs = []

    def close(self):
        self._offline_pool.shutdown(wait=True)
        self.reset()

    def _region_job(self, region, frame, color, source, host, current, zero):
        wall = time.perf_counter()
        cpu = time.thread_time()
        x0, y0, x1, y1 = region['bounds']
        crop = np.ascontiguousarray(color[y0:y1, x0:x1])
        visual = region['visual'].observe(crop, source, host)
        after_visual = time.perf_counter()
        anchor = region['visual']._anchor
        active = visual.get('qualified') is True and anchor is not None
        warming = visual.get('reason') == 'warming_up' and anchor is not None
        anchor_changed = region['anchor'] is not anchor
        reset = anchor_changed or not (active or warming)
        if reset:
            region['filter'] = None
            region['anchor'] = None
        applied = False
        owned_values = None
        if active or warming:
            region['anchor'] = anchor
            if region['filter'] is None:
                with self._offline_factory_lock:
                    created = self._factory(alpha=self.ALPHA, delta=self.DELTA,
                                            persistence=self.PERSISTENCE)
                    if created is None or any(created is r['filter'] for r in self._regions):
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
                # Advanced indexing detaches only this region's owned values.
                # Output composition remains on the caller in original order.
                owned_values = filtered[y0:y1, x0:x1][region['owned']]
                applied = True
            del result, filtered
        report = dict(name=region['name'], bounds_xyxy=list(region['bounds']),
            owned_pixels=int(np.count_nonzero(region['owned'])), qualified=active,
            warming=bool(warming and not active), history_reset=bool(reset),
            filter_applied=applied, visual=visual)
        return owned_values, report, dict(name=region['name'],
            wall_sec=time.perf_counter()-wall, cpu_sec=time.thread_time()-cpu,
            visual_wall_sec=after_visual-wall)

    def _parallel_regions(self, frame, color, source, host, current, zero, output, report):
        futures = [self._offline_pool.submit(self._region_job, region,
            frame, color, source, host, current, zero) for region in self._regions]
        # Even a worker failure must not race the outer process exception reset.
        wait(futures)
        results = [f.result() for f in futures]
        self.last_jobs = [row[2] for row in results]
        qualified = 0
        for region, (values, diagnostic, _) in zip(self._regions, results):
            if values is not None:
                x0, y0, x1, y1 = region['bounds']
                output[y0:y1, x0:x1][region['owned']] = values
                qualified += int(np.count_nonzero(region['owned']))
            report['regions'].append(diagnostic)
        return qualified


def _install_unchanged_prefix_suffix():
    original = textwrap.dedent(inspect.getsource(RegionalStationaryDepth.process))
    begin = original.index('        for region in self._regions:\n')
    end = original.index('        report.update(active=bool(qualified_pixels)', begin)
    replacement = ('        qualified_pixels = self._parallel_regions(\n'
                   '            frame, color, source, host, current, zero, output, report)\n')
    namespace = dict(np=np)
    exec(compile(original[:begin]+replacement+original[end:],
                 '<offline_parallel_original_prefix_suffix>', 'exec'), namespace)
    ParallelRegionalPrototype.process = namespace['process']
    return original


ORIGINAL_PROCESS_SOURCE = _install_unchanged_prefix_suffix()
