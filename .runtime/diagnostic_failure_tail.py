"""Passive telemetry tail after a failed bounded diagnostic.

Never clears a contract, invokes motion/settling code, or treats a failed step as
complete. The existing subscribers and healthy archive remain alive briefly.
"""
import time

DURATION_SECONDS = 5.0


def record_failure_tail(diagnostic, original_reason, *, monotonic=time.monotonic,
                        sleep=time.sleep, is_shutdown=lambda: False):
    result = {
        'original_failure': str(original_reason),
        'fault_cleared': False,
        'failed_command_completed': False,
        'requested_duration_seconds': DURATION_SECONDS,
        'status': 'not_started',
    }
    if diagnostic.publish_count == 0:
        result['status'] = 'not_needed_no_commands'
        return result
    # This disconnects this diagnostic's command publisher only. It sends no
    # position, stop, enable, disable, recovery, or return command to the driver.
    with diagnostic.lock:
        publisher = diagnostic.publisher
        diagnostic.publisher = None
        if not diagnostic.fault:
            diagnostic.fault = str(original_reason)
        result['entry_latched_fault'] = str(diagnostic.fault)
        initial_count = diagnostic.publish_count
    start = monotonic()
    try:
        if publisher is not None:
            publisher.unregister()
        result['publisher_disconnected'] = True
        diagnostic.archive_writer.require_healthy()
        diagnostic.event('passive_failure_tail_started', **result)
        accepted_sources, sdk_sources = set(), set()
        end = start + DURATION_SECONDS
        while monotonic() < end:
            if is_shutdown():
                result['status'] = 'interrupted_ros_shutdown'
                break
            diagnostic.archive_writer.require_healthy()
            with diagnostic.lock:
                if diagnostic.publisher is not None or diagnostic.publish_count != initial_count:
                    raise RuntimeError('command interface changed during passive failure tail')
                # Contract and fault remain latched; do not call guard(),
                # stationary(), snapshot(), or any execution method here.
                if diagnostic.accepted:
                    accepted_sources.add(diagnostic.accepted[-1].header.stamp.to_nsec())
                if diagnostic.reference:
                    sdk_sources.add(diagnostic.reference[-1].header.stamp.to_nsec())
            sleep(min(.02, max(0., end - monotonic())))
        else:
            result['status'] = 'passive_window_recorded'
        result.update(observed_latest_accepted_source_count=len(accepted_sources),
                      observed_latest_sdk_source_count=len(sdk_sources),
                      elapsed_seconds=monotonic()-start)
        diagnostic.event('passive_failure_tail_finished', **result)
    except BaseException as error:
        # Do not replace the original diagnostic failure with an archive/tail
        # failure. The caller still closes/drains the archive and reports both.
        result.update(status='passive_tail_incomplete', tail_failure=str(error),
                      elapsed_seconds=monotonic()-start)
    result['commands_during_tail'] = diagnostic.publish_count - initial_count
    return result
