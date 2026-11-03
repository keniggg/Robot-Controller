"""Twenty seconds of already published numeric timing, with no motion I/O."""
import json
import os
from pathlib import Path
import statistics
import threading
import time

import rospy
from std_msgs.msg import String

SESSION = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z')


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert json.loads((SESSION/'decode_timing_deployment/result.json').read_text())['deployment_passed']
    output = SESSION/'decode_component_observation.json'
    assert not output.exists()
    os.environ['ROS_LOG_DIR'] = str(SESSION/'decode_observer_ros')
    rospy.init_node('decode_component_observer_readonly', anonymous=True, disable_rosout=True)
    rows, lock = [], threading.Lock()
    def receive(msg):
        data = json.loads(msg.data)
        timing = data.get('frame_timing', {})
        if not timing.get('decode_components'):
            return
        row = dict(observed_ns=rospy.Time.now().to_nsec(), state=data.get('state'),
                   detail=data.get('detail'), generation=data.get('generation'),
                   source_stamp_ns=data.get('source_stamp_ns'), frame_timing=timing)
        with lock:
            if len(rows) < 300:
                rows.append(row)
    sub = rospy.Subscriber('/perception/unknown/detector_status', String, receive, queue_size=10)
    deadline = time.monotonic()+20.
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(.1)
    sub.unregister()
    with lock:
        result = dict(motion_commands_sent=False, image_time_bound=True,
                      observation_only=True, rows=list(rows), components={})
    for key in ('camera_parameter_read', 'rectified_source_validation',
                'cv_bridge_conversion', 'float32_depth_scale'):
        values = [r['frame_timing']['decode_components'][key] for r in result['rows']
                  if key in r['frame_timing']['decode_components']]
        if values:
            result['components'][key] = dict(count=len(values),
                median_wall_ms=statistics.median(v['wall_sec'] for v in values)*1000.,
                max_wall_ms=max(v['wall_sec'] for v in values)*1000.,
                max_thread_cpu_ms=max(v['thread_cpu_sec'] for v in values)*1000.)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), count=len(result['rows']),
                         components=result['components']), ensure_ascii=False))


if __name__ == '__main__':
    main()
