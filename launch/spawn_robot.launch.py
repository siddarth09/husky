import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
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
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time')
    controller_config_arg = DeclareLaunchArgument('controller', default_value=PathJoinSubstitution([pkg_husky_description, 'config', 'controller.yaml']), description='Controller YAML path')

    urdf_file_path = PathJoinSubstitution([pkg_husky_description, "urdf", LaunchConfiguration('model')])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_husky_description, 'launch', 'world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_husky_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "husky", "-topic", "robot_description", "-x", "0.0", "-y", "0.0", "-z", "0.2"],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

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
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    gz_bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
                "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
                "/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",

                # 🦶 Foot contact sensors:
                "/lf_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
                "/lh_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
                "/rf_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
                "/rh_foot/contacts@gazebo_msgs/msg/ContactsState[gz.msgs.Contacts",
            ],
            output="screen",
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ]
        )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            LaunchConfiguration('controller'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller'],
                output='screen'
            )
        ]
    )


    static_pose_node= Node(
        package='husky_controllers',
        executable='static_pose.py',
        name='static_pose',
        output='screen',
        
    )
    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(controller_config_arg)
    ld.add_action(gz_bridge_node)
    ld.add_action(gz_image_bridge_node)
    ld.add_action(relay_camera_info_node)
    ld.add_action(world_launch)
    ld.add_action(rviz_node)
    ld.add_action(spawn_urdf_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    # ld.add_action(static_pose_node)
    
    return ld