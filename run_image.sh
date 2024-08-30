#/bin/bash
xhost +local:docker
    docker run --rm -it --net=host --name robot_container --volume /tmp/.X11-unix:/tmp/.X11-unix --volume .:/MR_ROBOT_HARDWARE/src --workdir /MR_ROBOT_HARDWARE --env DISPLAY=$DISPLAY --env ROS_MASTER_URI=http://localhost:11311/ --privileged -v /dev/bus/usb:/dev/bus/usb --env ROS_HOSTNAME=192.168.0.101 --env ROS_IP=192.168.0.100 ros-noetic-env:1.1  /bin/bash -c "catkin_make && source devel/setup.bash && /bin/bash"
