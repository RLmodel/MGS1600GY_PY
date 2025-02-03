import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('mgs1600gy_py'),
        'config',
        'config.yaml'
        )
        
    mgs_node = Node(
        package='mgs1600gy_py',
        executable='mgs_serial_driver',
        # name='mgs_serial_driver',
        emulate_tty=True,
        output='screen',
        parameters=[config],
    )
       
    return LaunchDescription([
        mgs_node,
    ])