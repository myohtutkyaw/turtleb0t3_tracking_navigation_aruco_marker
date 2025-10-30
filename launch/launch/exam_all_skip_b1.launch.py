from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    params = os.path.join(
        get_package_share_directory('tb3_final_exam'),
        'config', 'exam_params.yaml'
    )

    exam_all = Node(
        package='tb3_final_exam',
        executable='exam_all',
        name='exam_all',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            # Skip B1 (SLAM wait) because you already launched Nav2+AMCL with a saved map
            {'slam.use_lifecycle': False,
             'slam.min_map_updates': 0,
             'slam.max_wait_s': 0.0}
        ]
    )

    return LaunchDescription([use_sim_time, exam_all])
