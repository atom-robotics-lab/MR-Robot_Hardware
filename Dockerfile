FROM ros:noetic-ros-core-focal

SHELL ["/bin/bash","-c"]

RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-catkin-tools \
    python3-vcstool \
    ros-noetic-catkin \
    ros-noetic-roslaunch \
    ros-noetic-ros-base \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-rosserial-arduino \
    ros-noetic-xacro  \
    git \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher
    

RUN rosdep init && rosdep update

WORKDIR /root/MR_ROBOT_HARDWARE/src

RUN git clone https://github.com/atom-robotics-lab/MR-Robot_Hardware.git -b kirinain/Docker


WORKDIR /root/MR_ROBOT_HARDWARE
RUN source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN echo "source /root/MR_ROBOT_HARDWARE/devel/setup.bash" >> /root/.bashrc



