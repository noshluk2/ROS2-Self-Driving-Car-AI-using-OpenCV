import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   # main package path
  package_dir=get_package_share_directory('tsc')
  world_file = os.path.join(package_dir,'self_driving.world')
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
  
  return LaunchDescription([
    #launching gazebo server and our custom world in it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items(),
        ),
    # Adding the gzclient launch file to run gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
    ])