# ROS2 Prius Self Driving Car  using AI/Deeplearning and Computer Vision

## This Repository and its Readme is UNDER CONSTRUCTION !

TODO : 
- Description
- Course Thumnbail
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About-this-Repository">About This Repository</a></li>
    <li><a href="#Using-this-Repository">Using this Repository</a></li>
    <li><a href="#Course-Workflow">Course Workflow</a></li>
    <li><a href="#Features">Features</a></li>
    <li><a href="#Pre-Course-Requirments">Pre-Course Requirments</a></li>
    <li><a href="#Link-to-the-Course">Link to the Course</a></li>
    <li><a href="#Instructors">Instructors</a></li>
    <li><a href="#License">License</a></li>
  </ol>
</details>

## About this Repository
A tesla Like Car in ROS2 will follow lane , Use AI to classify Sign Boards and perform Object tracking to act on the sign boards and set speed respectively

- ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/a.png)
----
## Using this Repository
* Clone the repository in you Home folder 
```
git clone https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV.git
```
* Get into the downloaded repository
 ```
 cd path/to/ROS2-Self-Driving-Car-AI-using-OpenCV/
##e.g cd ~/ROS2-Self-Driving-Car-AI-using-OpenCV/
  ```

* Bring all models into your **.gazebo/models**
 ```
 cp /models/* ~/.gazebo/models 
 ```
 or manually copy->paste them into ~/.gazebo/models/

* Perform Colcon Build
```
colcon build
```
* Source your Workspace in any terminal you open to Run files from this workspace ( Basics thing of ROS )
```
source /path/to/ROS2-Self-Driving-Car-AI-using-OpenCV/install/setup.bash
```
* (Optional for Power USERs ) Add source to this workspace into bash file
 ```
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 ```
  **NOTE:** This upper command is going to add the source file path into your ~/.bashrc file ( Only perform it once and you know what you are doing).This will save your time when running things from the Workspace
* If the repository is not working for you. Watch the free preview video on our course page 
 in **Section # 6 : How to use Code** where full explaination is given on setting up this repository.
  * **[[Link not added Yet !]]()**
----
## Course Workflow
#### **Ros Package**
* World Models Creation
* Prius OSRF gazebo Model Editing
* Nodes , Launch Files
* SDF through Gazebo
* Textures and Plugins in SDF

#### **Computer Vision** 
* @HaiderAbasi add your workflow
#### **DeepLearning**
* @HaiderAbasi add your workflow
---
## Features
* **Prius Hybrid Car** 
  -  ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/the_car.gif)
* **Lane Following** 
  -  ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/lane_detection.gif)
* **Sign Board Detection**
  - ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/traffic_signs_boards.gif)
* **Traffic Signal Recognition**
  - ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/traffic_signal.gif)

* **T-Junction Navigation**
  - ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/j_turning.gif)

* **The World** 
  -  ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/world.gif)

* **Custom Models** 
  -  ![alt text](https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/custom_models.gif)

----
## Pre-Course Requirments 

**Software Based**
* Ubuntu 20.04 (LTS)
* ROS2 - Foxy Fitzroy 
* Python 3.6 
* Opencv 4.2
* Tensorflow 2.14

**Skill Based**
* Basic ROS2 Nodes Communication
* Launch Files 
* Gazebo Model Creation
* @HaiderAbasi Add yours
* Motivated mind :)
---
## Link to the Course
<!-- - ![alt text](https://github.com/HaiderAbasi/SelfDrivingProject_MiniTesla/blob/master/3D%20model%20file/Tesla%20Self%20Driving%20Car.png) -->

**[[Course Not Yet Published]](https://www.udemy.com/user/e8894488-eb79-45f5-aef1-f3a8733b6f43/)**

----

## Instructors

Haider Najeeb   (Computer Vision)    -  [Profile Link](https://www.linkedin.com/in/haider-najeeb-68812516a/)  
Muhammad Luqman (ROS Simulation and Control Systems) - [Profile Link](https://www.linkedin.com/in/muhammad-luqman-9b227a11b/)  

----
## License

Distributed under the GNU-GPL License. See `LICENSE` for more information.
