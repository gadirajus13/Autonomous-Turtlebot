import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen'
        ),
        Node(
            package='autonomous_tb',
            executable='data_collection_node',
            name='data_collection_node',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()