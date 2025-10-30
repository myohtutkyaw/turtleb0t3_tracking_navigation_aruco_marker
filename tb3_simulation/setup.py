from setuptools import setup

package_name = 'tb3_final_exam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # <— explicit
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',]),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox.yaml',
            'config/nav2_params.yaml',
            'config/aruco.yaml',
            'config/exam_params.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/bringup_slam_nav.launch.py',
            'launch/task_a1_path_follow.launch.py',
            'launch/task_a2_qr_turn.launch.py',
            'launch/task_b2_nav_to_goal.launch.py',
            'launch/task_c_dock.launch.py',
            'launch/exam_all.launch.py',
            'launch/exam_all_skip_b1.launch.py',
            'launch/show_cam.launch.py',
        ]),
        ('share/' + package_name + '/resource', [
            'resource/' + package_name
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jimmy',
    maintainer_email='jimmy@todo.todo',
    description='Task-by-task TB3 final exam nodes (A1, A2, B1, B2, C) for ROS 2 Humble.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_a1_path_follow = tb3_final_exam.task_a1_path_follow:main',
            'task_a2_qr_turn    = tb3_final_exam.task_a2_qr_turn:main',
            'task_b2_nav_goal   = tb3_final_exam.task_b2_nav_to_goal:main',
            'task_c_dock        = tb3_final_exam.task_c_dock:main',
            'show_cam = tb3_final_exam.show_cam:main',
            'exam_all = tb3_final_exam.exam_all:main',
        ],
    },
)
