import os
import launch
import launch_ros.actions
import launch.substitutions
import yaml

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    param_config = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config', 'imu_filter.yaml')

    # https://github.com/ros2/rclcpp/issues/715#issuecomment-490425249
    # Composable Nodes use different yaml parsing than a standalone node.
    # This code will load the parameters from the yaml (removing the namespace/nodename/ros__parameters heading) so
    # that the parameters are parsed and named properly for the composable node.
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['imu_filter']['ros__parameters']

    container = ComposableNodeContainer(
        name='imu_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='imu_filter_madgwick',
                plugin='ImuFilterMadgwickRos',
                name='imu_filter',
                parameters=[params],
            )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
