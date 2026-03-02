#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    justina_share = get_package_share_directory('justina_description')
    world_path = os.path.join(justina_share, 'webots_env', 'worlds', 'configuring_robot.wbt')
    controllers_path = os.path.join(justina_share, 'webots_env', 'controllers')

    env = os.environ.copy()
    env['WEBOTS_CONTROLLER_PATH'] = controllers_path
    env['DISPLAY'] = os.environ.get('DISPLAY', ':0')

    webots_process = ExecuteProcess(
        cmd=['webots', '--mode=realtime', world_path],
        output='screen',
        env=env,
    )

    return LaunchDescription([webots_process])