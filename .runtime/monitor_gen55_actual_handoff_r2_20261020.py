"""Use the reviewed file-only GEN55 monitor with a separate second-attempt output."""
import hashlib
from pathlib import Path

import monitor_gen55_actual_handoff_20261019 as monitor


if __name__ == '__main__':
    original = Path(monitor.__file__)
    assert hashlib.sha256(original.read_bytes()).hexdigest() == '7000e2e38c468c4789aa7a37317d6733366960e6cf1fef04dc95515c9bec812a'
    monitor.OUTPUT = monitor.ORIGINAL.V / 'gen55_actual_handoff_monitor_r2'
    monitor.main()
