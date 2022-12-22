
## 1. Install Docker on Ubuntu 22.04
```
https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04
```
## 2. Setup Nvidia Runtime for faster execution
- Reference https://github.com/NVIDIA/nvidia-docker/issues/838

    ### Setting the GPG and remote repo for the package:
    ```
    curl -s -L nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    ```
    ```
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    ```
    ```
    curl -s -L nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    ```
    ### Save that file and refresh the package list
    ```
    sudo apt-get update
    ```
    ### Install nvidia-docker2 and reload the Docker configurations
    ```
    sudo apt-get install -y nvidia-docker2
    ```
    ```
    sudo pkill -SIGHUP dockerd
    ```
## 3. Pull Image from Docker-hub
- Get docker image
    ```
    docker pull noshluk2/ros2-self-driving-car-ai-using-opencv:latest
    ```
## 4. Create container from image
- Make bash script executable
    ```
    chmod +x create_container.bash
    ```
- Run it
    ```
    ./create_container.bash
    ```
- **IMPORTANT** : Do not run the bash script again and again . It will keep on creating multiple containers and we donot need that . Only one container is required.

## 5. Getting into container
- If container was stopped then
    ```
    docker start ros2_sdc_container
    ```
- Connecting a terminal with started container
    ```
    docker exec -it ros2_sdc_container bash
    ```
## 6. Launching Project
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
