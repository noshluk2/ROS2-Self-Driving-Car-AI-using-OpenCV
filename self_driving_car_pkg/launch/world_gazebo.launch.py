#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share_dir = get_package_share_directory('self_driving_car_pkg')
    model_pkg_share_dir = get_package_share_directory('self_driving_car_pkg_models')
    models_share_dir = os.pathsep + os.path.join(model_pkg_share_dir, 'models')


    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += models_share_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  models_share_dir
    print(os.environ['GAZEBO_MODEL_PATH'])

    world = os.path.join(
        pkg_share_dir,
        'worlds',
        'sdc.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    red_light_sdf = os.path.join(pkg_share_dir,'models','light_red/model.sdf')
    yellow_light_sdf = os.path.join(pkg_share_dir,'models','light_yellow/model.sdf')
    green_light_sdf = os.path.join(pkg_share_dir,'models','light_green/model.sdf')

    spawn_red_light = ExecuteProcess(
        cmd=['ros2', 'run', 'self_driving_car_pkg', 'spawner_node', red_light_sdf,'red_light'],
        output='screen'
    )

    spawn_yellow_light = ExecuteProcess(
        cmd=['ros2', 'run', 'self_driving_car_pkg', 'spawner_node', yellow_light_sdf,'yellow_light'],
        output='screen'
    )

    spawn_green_light = ExecuteProcess(
        cmd=['ros2', 'run', 'self_driving_car_pkg', 'spawner_node', green_light_sdf,'green_Light'],
        output='screen'
    )


    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ld.add_action(spawn_red_light)
    ld.add_action(TimerAction(
        period=7.5,
        actions=[spawn_yellow_light]
    ))
    ld.add_action(TimerAction(
        period=8.5,
        actions=[spawn_green_light]
    ))

    return ld