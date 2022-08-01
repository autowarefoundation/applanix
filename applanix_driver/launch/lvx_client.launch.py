import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('applanix_driver'),
        'config',
        'lvx_client_params.param.yaml'
    )

    # XXX The node name has to match the one in the yaml
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='lvx_client',
            description='Name of the node'
        ),
        DeclareLaunchArgument(
            'config',
            default_value=config,
            description='YAML file for setting ROS params'
        ),
        Node(
            package='applanix_driver',
            executable='lvx_client_node',
            name=LaunchConfiguration('node_name'),
            parameters=[config, LaunchConfiguration('config')],
            output='screen'
        )
    ])

    return ld
