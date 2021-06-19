import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('autonomous_rover')
    urdf = os.path.join(package_dir,'rover_meshed.urdf')
    rviz_config_file=os.path.join(package_dir, 'view.rviz')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config_file],
        output='screen'),
        

    ])