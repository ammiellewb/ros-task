from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Gazebo Server (headless)
    gz_server = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-s', '-v', '4', '-r', 'empty.sdf'],
        output='screen',
        shell=True
    )
    ld.add_action(gz_server)
    
    # Gazebo GUI (connect to server)
    gz_gui = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-g', '-v', '4', '--connect', '127.0.0.1'],
        output='screen',
        shell=True
    )
    ld.add_action(gz_gui)
    
    # Controller Node (start after Gazebo is ready)
    controller = Node(
        package='limo_control',
        executable='pose_controller',
        name='pose_controller',
        output='screen'
    )
    
    # Delay controller start until Gazebo is ready
    delayed_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_server,
            on_exit=[controller]
        )
    )
    ld.add_action(delayed_controller)
    
    return ld