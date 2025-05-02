import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_husky_description = get_package_share_directory('husky_description')

    # Set Gazebo model path
    gazebo_models_path = os.path.join(pkg_husky_description)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path

    # Launch arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Open RViz')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='urdf.rviz', description='RViz config file')
    world_arg = DeclareLaunchArgument('world', default_value=PathJoinSubstitution([pkg_husky_description, 'worlds', 'empty.sdf']), description='Path to Gazebo world')
    model_arg = DeclareLaunchArgument('model', default_value='husky_description.xacro', description='URDF filename')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    controller_config_arg = DeclareLaunchArgument('controller', default_value=PathJoinSubstitution([pkg_husky_description, 'config', 'controller.yaml']), description='Controller YAML path')

    urdf_file_path = PathJoinSubstitution([pkg_husky_description, "urdf", LaunchConfiguration('model')])

    # Launch world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_husky_description, 'launch', 'world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(Command(['xacro', ' ', urdf_file_path]), value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # Spawn robot in Gazebo
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "husky", "-topic", "robot_description", "-x", "0.0", "-y", "0.0", "-z", "0.08"],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Bridge
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/lf_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
            "/lh_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
            "/rf_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
            "/rh_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
        ],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Optional RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_husky_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ros2_control manager
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            LaunchConfiguration('controller'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Position controller spawner
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen'
    )

    
    pose_commander_node = Node(
        package='husky_controllers',
        executable='pose.py',
        name='pose_commander',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        rviz_arg,
        rviz_config_arg,
        world_arg,
        model_arg,
        sim_time_arg,
        controller_config_arg,
        world_launch,
        gz_bridge_node,
        rviz_node,
        robot_state_publisher_node,
        spawn_urdf_node,
        ros2_control_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_urdf_node,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[position_controller_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=position_controller_spawner,
                on_exit=[pose_commander_node]
            )
        )
    ])
