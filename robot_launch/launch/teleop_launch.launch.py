import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Motor driver node
    robot_motors_node=Node(
        package="robot_motors",
        executable="Motor_node",
    )

    # twist_mux node
    twist_mux_config = os.path.join(get_package_share_directory('robot_launch'),
                                        'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        parameters=[
            {
                'use_sim_time': False,
                'use_stamped': False
            },
            twist_mux_config]
    )

    ld.add_action(robot_motors_node)
    ld.add_action(twist_mux)

    return ld