"""Compatibility entrypoint for the guarded, auditable retry preparation tool.

A reason alone can no longer reset a trial after possible observation motion.
Default invocation is read-only; see --alignment-receipt and --apply.
"""
from pathlib import Path
import runpy

runpy.run_path(str(Path(__file__).resolve().parents[1] /
    'tools/diagnostics/prepare_rgb_acquisition_retry.py'), run_name='__main__')
