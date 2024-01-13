"""
VoronoiPlanner app launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

January 11, 2022
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Builds a LaunchDescription for the Joy2cmdvel app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(get_package_share_directory('voronoi_planner'), 'config/voronoi_planner.yaml')

    # Create node launch description
    node = Node(
        package='voronoi_planner',
        executable='voronoi_planner',
        exec_name='voronoi_planner_app',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config]
    )

    ld.add_action(node)

    return ld
