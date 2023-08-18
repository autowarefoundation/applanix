# Just an example composition launch file to show that lvx_client is composable.
# This file is mostly useless on its own.
import launch
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('applanix_driver'),
        'config',
        'lvx_client_params.param.yaml'
    )

    arg_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='lvx_client',
        description='Name of the node'
    )

    arg_config = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='YAML file for setting ROS params'
    )

    # Parse default config file into parameter dictionary
    with open(default_config, 'r') as f:
        default_params = yaml.safe_load(f)['lvx_client']['ros__parameters']

    container = ComposableNodeContainer(
            node_name='applanix_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='applanix_driver',
                    node_plugin='applanix_driver_ros::LvxClientRos',
                    node_name=LaunchConfiguration('node_name'),
                    parameters=[default_params, LaunchConfiguration('config')])
            ],
            output='screen',
    )

    return launch.LaunchDescription([arg_node_name, arg_config, container])
