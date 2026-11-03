"""Load the workspace's shared disk selector without ROS or path mutation."""
from functools import lru_cache
from pathlib import Path
import types


@lru_cache(maxsize=1)
def storage_module():
    source = Path(__file__).resolve().parents[5]/'tools/experiment_storage.py'
    module = types.ModuleType('_alicia_runtime_experiment_storage')
    module.__file__ = str(source)
    exec(compile(source.read_bytes(), str(source), 'exec'), module.__dict__)
    return module


def select_storage(backend=None, *, path=None):
    return storage_module().select(backend, path=path)
