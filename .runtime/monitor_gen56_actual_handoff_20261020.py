"""Read GEN56 consumer logs through the unchanged, pinned handoff observer."""
import hashlib
from pathlib import Path

import monitor_gen55_actual_handoff_20261019 as monitor


if __name__ == '__main__':
    assert hashlib.sha256(Path(monitor.__file__).read_bytes()).hexdigest() == '7000e2e38c468c4789aa7a37317d6733366960e6cf1fef04dc95515c9bec812a'
    monitor.CONSUMERS = monitor.ORIGINAL.N / 'gen56_runtime/compute_gateway_deployment_r1/consumers.log'
    monitor.OUTPUT = monitor.ORIGINAL.V / 'gen56_actual_handoff_monitor_r1'
    monitor.main()
