import xacro
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import EmitEvent
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from launch.actions import GroupAction
from launch_ros.actions import SetRemap



def generate_launch_description():
    ld = LaunchDescription()

    descr_dir = get_package_share_directory('robot_description')
    
    xacro_file = os.path.join(descr_dir, 'urdf', 'robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file).toxml()
    

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
    )

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
    
    robot_logic_node=Node(
        package="robot_logic",
        executable="StateMachineNode",

    )
    robot_motors_node=Node(
        package="robot_motors",
        executable="Motor_node",
    )

    robot_sensors_node=Node(
        package="robot_sensors",
        executable="ImuNode",
    )
    robot_sensors_uart_node=Node(
        package="robot_sensors",
        executable="sensor_input_uart_node",
    )

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan_raw'},
            {'frame_id': 'lidar_link'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
            {'resolution_fixed': True}
        ]
    )

    filter_config = os.path.join(
        get_package_share_directory('robot_launch'),
        'config',
        'scan_filter.yaml'
    )

    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[filter_config],
        remappings=[
            ('scan', 'scan_raw'),     # Input
            ('scan_filtered', 'scan') # Output for SLAM
        ]
    )


    ros2_laser_scan_matcher = Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen',
            parameters=[{

                'publish_tf': True,
                'publish_odom': 'odom_scan',
                # You can add other parameters here, like frames:
                'base_frame': 'base_footprint',
                'laser_frame': 'lidar_link',
                'map_frame': 'map',
                'max_iterations': 10
            }]
    )

    robot_joint_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
    )

    slam_config_path = os.path.join(
        get_package_share_directory('robot_launch'), # Or use a direct path
        'config',
        'slam_config.yaml'
    )

    fusion_config = os.path.join(
        get_package_share_directory('robot_launch'), # Or use a direct path
        'config',
        'ekf.yaml'
    )

    nav2_config_path = os.path.join(
        get_package_share_directory('robot_launch'), # Or use a direct path
        'config',
        'nav2_params.yaml'
    )

    slam_async_node = LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path],
            namespace=''
    )

    configure_event = EmitEvent(
    event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_async_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )   

    activate_event = EmitEvent(
    event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_async_node),
        transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_async_node,
            goal_state='inactive',
            entities=[activate_event],
        )
    )

    pkg_dir = get_package_share_directory("robot_launch")

    nav2_launch = GroupAction([
        #SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'), if using twist_mux, remap the input to nav2 to avoid conflicts with robot_logic

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "launch", "navigation.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": nav2_config_path,
            }.items(),
        )
    ])

    delayed_nav2 = TimerAction(
    period=8.0,  # Delay in seconds (adjust as needed)
    actions=[nav2_launch]
    )


    fusion_node = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node",
    output="screen",
    parameters=[fusion_config]
    )

    explore_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("mrtsp_exploration_ros2"),
            "launch",
            "explore.launch.py",
        )
    ),
)
    

    delayed_nodes = TimerAction(
        period=6.0,  # Delay in seconds (adjust as needed)
        actions=[fusion_node, configure_event, slam_async_node, activate_on_configure]#add twist mux here if you want to delay it as well
    )

    delayed_exploration = TimerAction(
        period=8.0,  # Delay in seconds (adjust as needed)
        actions=[explore_launch]
    )
    
    
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_joint_publisher)
    ld.add_action(ldlidar_node)
    ld.add_action(ros2_laser_scan_matcher)
    ld.add_action(scan_filter_node)
    #ld.add_action(robot_logic_node)
    ld.add_action(robot_motors_node)
    ld.add_action(robot_sensors_uart_node)
    ld.add_action(robot_sensors_node)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_nodes)
    #ld.add_action(delayed_exploration)
    return ld




    