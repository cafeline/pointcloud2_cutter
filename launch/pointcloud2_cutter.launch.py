from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud2_cutter')
    param_file = os.path.join(package_share, 'config', 'pointcloud2_cutter.param.yaml')

    pointcloud2_cutter_node = Node(
        package='pointcloud2_cutter',
        executable='pointcloud2_cutter_node',
        name='pointcloud2_cutter',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([pointcloud2_cutter_node])
