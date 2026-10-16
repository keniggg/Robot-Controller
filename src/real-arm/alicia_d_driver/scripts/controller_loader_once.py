#!/usr/bin/env python3
"""Load and start ros_control controllers without a shutdown stop hook."""

import sys

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    SwitchController,
    SwitchControllerRequest,
)


def _controller_states(list_controllers):
    return {
        controller.name: controller.state
        for controller in list_controllers().controller
    }


def main():
    rospy.init_node('controller_loader_once')
    controller_names = [
        name for name in rospy.myargv(argv=sys.argv)[1:] if name.strip()
    ]
    if not controller_names:
        rospy.logerr('No controller names were supplied')
        return 2

    load_service = '/controller_manager/load_controller'
    list_service = '/controller_manager/list_controllers'
    switch_service = '/controller_manager/switch_controller'
    for service in (load_service, list_service, switch_service):
        rospy.wait_for_service(service, timeout=30.0)

    load_controller = rospy.ServiceProxy(load_service, LoadController)
    list_controllers = rospy.ServiceProxy(list_service, ListControllers)
    switch_controller = rospy.ServiceProxy(switch_service, SwitchController)

    states = _controller_states(list_controllers)
    for name in controller_names:
        if name in states:
            continue
        response = load_controller(name)
        if not response.ok:
            rospy.logerr('Failed to load controller %s', name)
            return 3

    states = _controller_states(list_controllers)
    to_start = [
        name for name in controller_names if states.get(name) != 'running'
    ]
    if to_start:
        response = switch_controller(
            to_start,
            [],
            SwitchControllerRequest.STRICT,
            False,
            0.0,
        )
        if not response.ok:
            rospy.logerr('Failed to start controllers: %s', ', '.join(to_start))
            return 4

    rospy.loginfo(
        'Controllers loaded and running; one-shot loader is exiting without '
        'installing a shutdown stop/unload hook: %s',
        ', '.join(controller_names),
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
