xhost local:root


XAUTH=/tmp/.docker.xauth

docker run -it \
    --name=ros2_sdc_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --runtime=nvidia \
    noshluk2/ros2-self-driving-car-ai-using-opencv \
    bash

echo "Done."