#1/bin/bash
xhost +local:docker
docker run --rm -it --net=host --name robot_container --volume /tmp/.X11-unix:/tmp/.X11-unix --volume .:/MR_ROBOT_HARDWARE/src --workdir /MR_ROBOT_HARDWARE --env DISPLAY=$DISPLAY --env ROS_MASTER_URI=http://localhost:11311/ --env ROS_HOSTNAME=192.168.100.12 --env ROS_IP=192.168.100.12 ros-noetic-env /bin/bash