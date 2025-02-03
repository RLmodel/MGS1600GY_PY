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
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='log',
        emulate_tty=True,
    )
    
    
    mgs_node = Node(
        package='mgs1600gy_py',
        executable='mgs_serial_driver',
        # name='mgs_serial_driver',
        emulate_tty=True,
        output='log',
        parameters=[config],
    )
    
    cmd_vel_node = Node(
        package='mgs1600gy_py',
        executable='mgs_control',
        # name='mgs_control',
        emulate_tty=True,
        output='log',
        remappings=[
            ('cmd_vel', 'turtle1/cmd_vel'),
        ]        
        )        
    
    
    
    return LaunchDescription([
        turtlesim_node,
        mgs_node,
        cmd_vel_node,
    ])