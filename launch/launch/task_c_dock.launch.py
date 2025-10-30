from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value='/ABSOLUTE/PATH/TO/Stadium.world')

    pkg_share = get_package_share_directory('tb3_final_exam')

    bringup_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'bringup_world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # optional: if you need map/TFs while docking
    bringup_slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'bringup_slam_nav.launch.py'))
    )

    exam_params = os.path.join(pkg_share, 'config', 'exam_params.yaml')

    cdock = Node(
        package='tb3_final_exam',
        executable='task_c_dock',
        name='task_c_dock',
        parameters=[exam_params],
        output='screen'
    )

    return LaunchDescription([world_arg, bringup_world, bringup_slam_nav, cdock])
