
## Pre-Requisites Installation:
1. Get Docker desktop installed and running by following this [Guide](https://docs.docker.com/desktop/install/windows-install/)
2. Ensure WSL-2 is installed or upgraded from WSL-1 (if you have older version).
3. Install VcXsrv Windows X Server from this [link](https://sourceforge.net/projects/vcxsrv/)


## 1. Pull Image from Docker-hub
- Get docker image
    ```
    docker pull noshluk2/ros2-self-driving-car-ai-using-opencv:latest
    ```
## 2. Create container from image
- >    ( For Cuda-enabled Graphic cards ) **Faster**
   ```
   docker run -it --name ros2_sdc_container -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 --runtime=nvidia noshluk2/ros2-self-driving-car-ai-using-opencv bash
   ```
- >   ( For Non-Cuda (Amd or CPU)      ) **Slower!**
   ```
   docker run -it --name ros2_sdc_container -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 noshluk2/ros2-self-driving-car-ai-using-opencv bash
   ```
- **IMPORTANT** : Do not run upper command again and again . It will keep on creating multiple containers and we donot need that . Only one container is required.
-   To enter an **already running container** run,

## 3. Getting into container
- If container was stopped then
    ```
    docker start ros2_sdc_container
    ```
- Connecting a terminal with started container
    ```
    docker exec -it ros2_sdc_container bash
    ```

## 4. Launching Project
1.  Bringing the Environment with Prius Car
      ```
      ros2 launch self_driving_car_pkg world_gazebo.launch.py
      ```
2. Open another terminal and connect it to container
   ```
    docker exec -it ros2_sdc_container bash
   ```
3. Move inside of workspace
   ```
   cd ~/ROS2-Self-Driving-Car-AI-using-OpenCV/
   ```
4. Run Self Driving node
   ```
   ros2 run self_driving_car_pkg computer_vision_node
   ```
