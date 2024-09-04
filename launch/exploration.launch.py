import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Set the config directory
    config_dir = os.path.join(get_package_share_directory('autonomous_tb'), 'config')
    map_dir = os.path.join(get_package_share_directory('autonomous_tb'), 'maps')
    # Use simulation time
    use_sim_time = SetEnvironmentVariable('use_sim_time', 'true')

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/turtlebot3_house.launch.py']),
        launch_arguments={
            'x_pose': '-2.0',
            'y_pose': '1.25'
        }.items(),
    )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('turtlebot3_gazebo'), 'launch'),
    #         '/turtlebot3_world.launch.py'])
    # )


    # Include the Cartographer launch file
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_cartographer'), 'launch'),
            '/cartographer.launch.py']),
        launch_arguments={'use_sim_time': 'true',
                          'log_level': 'warn'}.items(),
    )

    # Include the Nav2 launch file
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir, 'bringup_launch.py')]),
        launch_arguments={
            'map': 'none',
            'use_sim_time': 'true',
            'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
            'autostart': 'true'
        }.items(),
    )

    # Launch the exploration node
    exploration_node = Node(
        package='autonomous_tb',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(use_sim_time)
    ld.add_action(gazebo)
    ld.add_action(cartographer)
    ld.add_action(nav2)
    ld.add_action(exploration_node)

    return ld