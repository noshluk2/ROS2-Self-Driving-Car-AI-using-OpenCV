from inspect import Arguments
import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('self_driving_car_pkg')
    models_directory = os.path.join(package_dir,'models')
    # sign_30_sdf=os.path.join(models_directory,'sign_board_30','model.sdf')
    sign_30_sdf="/home/luqman/ros2_workspace/src/self_driving_car_pkg/models/sign_board_30/model.sdf"
    sign_turn_sdf="/home/luqman/ros2_workspace/src/self_driving_car_pkg/models/sign_board_turn/model.sdf"
    sign_stop_sdf="/home/luqman/ros2_workspace/src/self_driving_car_pkg/models/sign_board_stop/model.sdf"
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),


    ])