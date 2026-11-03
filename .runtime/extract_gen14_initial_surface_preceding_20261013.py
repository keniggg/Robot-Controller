"""Export historical preceding accepted sources; fusion membership remains to verify."""
import importlib.util
import json
from pathlib import Path

path = Path(__file__).with_name('extract_gen14_initial_surface_20261013.py')
spec = importlib.util.spec_from_file_location('original_initial_surface_export', path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
root = module.F/'initial_surface_preceding'; root.mkdir(exist_ok=False)
rows = []
for source in (1791860167282787322, 1791860167481009244):
    module.NS = source
    module.O = root/str(source)
    module.main()
    rows.append(dict(source_ns=source, manifest=str(module.O/'manifest.json'), sha256=module.sha(module.O/'manifest.json')))
result = dict(scope='Historical exact preceding accepted sources; snapshot fusion membership is not inferred solely from stamp proximity',
    rows=rows, extraction_script_sha256=module.sha(path), wrapper_sha256=module.sha(__file__),
    no_ros_master=True, motion_commands_sent=False)
(root/'result.json').write_text(json.dumps(result, indent=2)+'\n')
