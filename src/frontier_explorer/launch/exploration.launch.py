import os
from ament_python import get_package_share_directory  # noqa
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    tb3_sim = get_package_share_directory('turtlebot3_gazebo')
    slam_pkg = get_package_share_directory('slam_toolbox')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # 1. Gazebo + TurtleBot3
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_sim, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 2. SLAM Toolbox (online async)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4. Frontier Explorer node (delayed 10s to let Nav2 start)
    explorer = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='frontier_explorer',
                executable='frontier_node',
                name='frontier_explorer',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 5. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        slam,
        nav2,
        explorer,
        rviz,
    ])
