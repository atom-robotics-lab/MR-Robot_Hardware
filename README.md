<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true">
    <img src="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true" alt="Logo" width="120" height="120">
  </a>

<h3 align="center">MR Robot</h3>

  <p align="center">
    This is the repo for the <a href="https://github.com/atom-robotics-lab/MR-Robot">MR-Robot: ModulaR Robot</a> Project, Mr robot is autonomous navigation robot made by A.T.O.M Robotics capable of doing multiple day to day operations such as mapping, navigation for transportation, sanitaion etc. Various other operations can also be performed thanks to its modularity.
    If you’re interested in helping to improve our Project</a>, find out how to <a href="https://github.com/atom-robotics-lab/MR-Robot/blob/main/contributing.md">contribute<a>. 
    <br />
    <a href=""><strong>Demo video »</strong></a>
    <br />
    <br />
    <a href="https://github.com/atom-robotics-lab/MR-Robot/issues/new?labels=bug&assignees=jasmeet0915,Kartik9250,insaaniManav,namikxgithub">Report Bug</a>
    ·
    <a href="https://github.com/atom-robotics-lab/MR-Robot/issues/new?labels=enhancement&assignees=jasmeet0915,Kartik9250,insaaniManav,namikxgithub">Request Feature</a>
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project

This is the repo for the <a href="https://github.com/atom-robotics-lab/MR-Robot">MR-Robot: ModulaR Robot</a> Project, Mr robot is autonomous navigation robot made by A.T.O.M Robotics capable of doing multiple day to day operations such as mapping, navigation for transportation, sanitaion etc. Various other operations can also be performed thanks to its modularity.
If you’re interested in helping to improve our Project</a>, find out how to <a href="https://github.com/atom-robotics-lab/MR-Robot/contributing.md">contribute<a>.   

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

* [![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.sphinx-docs.org)
* [![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)
* [![Raspberry Pi](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)](https://www.raspberrypi.org/)
* [![Espressif](https://img.shields.io/badge/espressif-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)
* [![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)](https://www.arduino.cc/)
* [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
* [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

# DFD

<img src = "https://github.com/atom-robotics-lab/MR-Robot_Hardware/blob/main/assets/Mr_robot_dfd_1.png" >

# MR-Robot Circuit Diagram

<img src =  "https://github.com/atom-robotics-lab/MR-Robot_Hardware/blob/main/assets/MR_robot_circuit_1.png" >

# CONNECTION BETWEEN COMPUTER AND PI 
Now we have to setup a connection between the PC (Master) and the Pi(slave) for that we need to host the ip of the PC and get the Pi to connect to it.
For this get inside of the bashrc of the PC using command.
```shell
gedit .bashrc
```
And add the following lines
```shell
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=IP Address of PC
export ROS_IP= IP Address of PC
```
Now navigate into the pi and use the command
```shell
export ROS_MASTER_URI=http://IP Address of PC:11311/
export ROS_HOSTNAME=IP Address of PI
export ROS_IP=IP Address of PI
```
# ASSIGNING PORT FOR LIDAR AND ESP

It is crucial to assign ports for LIDAR and ESP because both of them receive random ports in ttyUSB format. 

Use This command to identify port and characters after "usb"

```shell
dmesg | grep ttyUSB
```
<img src = "https://github.com/atom-robotics-lab/MR-Robot_Hardware/blob/main/assets/usb.png" >

For creating the link from USB ports to the devices:-

```shell
sudo nano /etc/udev/rules.d/10-usb-serial.rules
```

```shell
SUBSYSTEM=="tty", KERNELS=="1-1.4", SYMLINK+="ttyUSB_ESP"
SUBSYSTEM=="tty", KERNELS=="1-1.1", SYMLINK+="ttyUSB_LIDAR"
```
<img src = "https://github.com/atom-robotics-lab/MR-Robot_Hardware/blob/main/assets/file.png" >

For loading the new rules:-

```shell
sudo udevadm trigger
```

By running the next command in the Terminal, you may verify the new names you just made:-

```shell
ls -l /dev/ttyUSB*
```

<img src = "https://github.com/atom-robotics-lab/MR-Robot_Hardware/blob/main/assets/list.png" >

# LAUNCHING LIDAR

```shell
roslaunch ydlidar_ros X2L.launch
```

This command will start ***"/scan"*** topic which will provide lidar data to pi which ultimately reaches Navigation Stack.

# ESP32 USAGE

ESP32 is controlling all the moter realted task in the bot it is receiving value of PWM from topic ***"/left_pwm"*** and ***"/right_pwm"*** which is then given to motor driver.

ESP32 receiving feedback data form motor's encoder by subscribing to the topic ***"/left_encoder"*** and ***"/right_encoder"*** which is then used in calculation odometry by pi.

# Launching Odom Calculation Node

```shell
rosrun mr_robot_firmware diff_tf.py
```

This node calculates odom from left encoder data and right encoder data.

# Launching Twist To PWM Node

```shell
rosrun mr_robot_firmware twist_to_pwm.py
```

This node converts Twist data into a signal using PWM for ESP32.
<!-- CONTACTS -->
## Contacts

Our Socials - [Linktree](https://linktr.ee/atomlabs) - atom@inventati.org



<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS-->
## Acknowledgement

* [Our wiki](https://atom-robotics-lab.github.io/wiki)
* [ROS Official Documentation](http://wiki.ros.org/Documentation)
* [Opencv Official Documentation](https://docs.opencv.org/4.x/)
* [Rviz Documentation](http://wiki.ros.org/rviz)
* [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
* [Ubuntu Installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
* [Raspberrypi Documentation](https://www.raspberrypi.com/documentation/)
* [Esp32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
* [Blender Documentaion](https://docs.blender.org/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/atom-robotics-lab/MR-Robot_Hardware.svg?style=for-the-badge
[contributors-url]: https://github.com/atom-robotics-lab/MR-Robot_Hardware/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/atom-robotics-lab/MR-Robot_Hardware.svg?style=for-the-badge
[forks-url]: https://github.com/atom-robotics-lab/MR-Robot_Hardware/network/members
[stars-shield]: https://img.shields.io/github/stars/atom-robotics-lab/MR-Robot_Hardware.svg?style=for-the-badge
[stars-url]: https://github.com/atom-robotics-lab/MR-Robot_Hardware/stargazers
[issues-shield]: https://img.shields.io/github/issues/atom-robotics-lab/MR-Robot_Hardware.svg?style=for-the-badge
[issues-url]: https://github.com/atom-robotics-lab/MR-Robot_Hardware/issues
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/company/a-t-o-m-robotics-lab/
