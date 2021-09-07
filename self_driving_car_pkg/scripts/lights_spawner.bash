#!/bin/bash
### This bash is going to spawn Traffic Lights into your simulation at origin 
### Inside the traffic stand
green_light_sdf=$HOME/.gazebo/models/green_light/model.sdf
red_light_sdf=$HOME/.gazebo/models/red_light/model.sdf
yellow_light_sdf=$HOME/.gazebo/models/yellow_light/model.sdf


ros2 run self_driving_car_pkg spawner_node $red_light_sdf red_light 
sleep 7.5
ros2 run self_driving_car_pkg spawner_node $yellow_light_sdf yellow_light 
sleep 1
ros2 run self_driving_car_pkg spawner_node $green_light_sdf green_Light 

