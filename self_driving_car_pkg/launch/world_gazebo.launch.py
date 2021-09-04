import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  package_dir=get_package_share_directory('self_driving_car_pkg')
  world_file = os.path.join(package_dir,'worlds','self_driving_car.world')

  return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose',world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),


        Node(
                package='self_driving_car_pkg',
                executable='lights_spawner.bash',
                name='Lights_installer',
                output='screen'),


        

    ])