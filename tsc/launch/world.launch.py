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
  world_file = os.path.join(package_dir,'car.world')
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
    #   publishes TF for links of the robot without joints
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     arguments=[urdf]),
    #  To publish tf for Joints only links
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     ),
    # to spawn the robot we added gazebo_ros spawn entity node
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "rehri"]
            ),
    # The last thing is Rviz with a configuration file
        # Node(
        # package='rviz2',
        # executable='rviz2',
        # name='rviz2',
        # arguments=['-d',rviz_config_file],
        # output='screen'),

  ])