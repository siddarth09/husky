import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_husky_description = get_package_share_directory('husky_description')

    # Ensure Gazebo can find Husky's model and world
    gazebo_models_path = os.path.join(pkg_husky_description)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path

    # Launch arguments
    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='false', description='Open RViz')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='urdf.rviz', description='RViz config file')
    world_arg = DeclareLaunchArgument('world', default_value=PathJoinSubstitution([pkg_husky_description, 'worlds', 'empty.sdf']), description='Path to the Gazebo world file')
    model_arg = DeclareLaunchArgument('model', default_value='husky_description.urdf', description='Name of the URDF description to load')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Flag to enable use_sim_time')

    # Path to the URDF file
    urdf_file_path = PathJoinSubstitution([pkg_husky_description, "urdf", LaunchConfiguration('model')])

    # Debug print to ensure URDF path is correct
    print(f"Using URDF file: {urdf_file_path}")

    # Include Gazebo world launch
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_husky_description, 'launch', 'world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_husky_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn the URDF model using Gazebo
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "husky",
            "-topic", "robot_description",  # Missing comma was here
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.9"
        ],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    # Bridge topics between Gazebo and ROS 2
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            
            
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            # "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            # "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            # "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            # "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            # "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            # "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            # "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # # Bridge camera image topic
    # gz_image_bridge_node = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image"],
    #     output="screen",
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'camera.image.compressed.jpeg_quality': 75}]
    # )

    # # Relay node to republish /camera/camera_info to /camera/image/camera_info
    # relay_camera_info_node = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='relay_camera_info',
    #     output='screen',
    #     arguments=['camera/camera_info', 'camera/image/camera_info'],
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(Command(['xacro'," ", urdf_file_path]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )


    # Launch description object
    launchDescriptionObject = LaunchDescription()

    # Add launch arguments
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(sim_time_arg)

    # Add launch nodes
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    # launchDescriptionObject.add_action(gz_bridge_node)
    # launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)

    return launchDescriptionObject
