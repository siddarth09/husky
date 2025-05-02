import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    husky_description_path = get_package_share_directory('husky_description')
    urdf_file = os.path.join(husky_description_path, 'urdf', 'husky_description.urdf')
    rviz_config = os.path.join(husky_description_path, 'urdf.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Robot model argument'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
