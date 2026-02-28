from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('webots_env')
    world_path = os.path.join(pkg_share, 'worlds', 'Justina_lab.wbt')
    controller_path = os.path.join(pkg_share, 'controllers', 'justina', 'justina.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['webots', '--stdout', '--stderr', world_path],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', controller_path],
            output='screen'
        )
    ])