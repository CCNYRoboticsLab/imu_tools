import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('imu_filter_madgwick'), 'config')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='imu_filter_madgwick', node_executable='imu_filter_madgwick_node', node_name='imu_filter', output='screen',
            parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
            remappings={('/imu_filter/imu/data_raw', '/imu_node/imu'), ('/imu_filter/imu/mag', '/imu_node/mag')}
        ),
        launch_ros.actions.Node(
            package='mscip', node_executable='imu_node', node_name='imu_node', output='screen',
            parameters=[os.path.join(get_package_share_directory('mscip'), 'config', 'imu.yaml')]
        )        
    ])
