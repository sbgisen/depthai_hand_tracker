#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_hand_tracker',
            executable='depthai_hand_tracker_ros.py',
            name='depthai_hand_tracker',
            output='screen'
        )
    ])
