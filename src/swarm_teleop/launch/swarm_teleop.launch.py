from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('swarm_teleop')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node(
            package='swarm_teleop',
            executable='teleop_control',
            name='teleop_control',
            prefix='gnome-terminal --',
        ),
        Node(
            package='swarm_teleop',
            executable='velocity_control',
            name='velocity'
        ),
        
    ])