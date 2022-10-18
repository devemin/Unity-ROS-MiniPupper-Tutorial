# Unity-ROS-MiniPupper-Tutorial
Unity x ROS1 x Mini Pupper Tutorial.

<br>

### (Important!) - You will need to install and Register Unity 3D Software beforehand! (You can use Free version.) 
### 'Unity Hub' is recommended.

<br>

https://unity3d.com/get-unity/download


<br>
<br>


# Overview

You can play your Mini Pupper with <b>Unity 3D Engine and ROS</b>!

Unity 3D Engine can also create <b>Windows / MacOS / Android / iOS Apps</b> and so on.

This repository contain a tutorial for connecting Unity and ROS.


![gif](https://github.com/devemin/Unity-ROS-MiniPupper-Tutorial/blob/main/media/overview.gif)

![image1](https://github.com/devemin/Unity-ROS-MiniPupper-Tutorial/blob/main/media/pic1.png)

![image2](https://github.com/devemin/Unity-ROS-MiniPupper-Tutorial/blob/main/media/pic2.png)


# Preparations

You need a Mini Pupper and PC.

## 1. Mini Pupper

(SD Card image: (64bit)v1.0.0.20220219.MiniPupper_ROS&OpenCV_Ubuntu20.04.03.img.zip )

https://drive.google.com/drive/folders/1RXu8dGXx3duYmF6jz4DYGml95TujSjsq

- Ubuntu 20.04
- ROS Noetic

ID: ubuntu / pass: mangdang

- Clone the ROS-TCP-Endpoint repository.
- (New image (20221013_minipupper_roscon2022_trim.img.xz), not need this.)


```
sudo apt update

cd ~/catkin_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ./ROS-TCP-Endpoint
pip3 install -r requirements.txt
cd ../
catkin_make

# for changing default python version 3
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.8 0
```



## 2. PC

Windows (10 or later)
or Ubuntu (I tried Ubuntu20.04)

<br>

### Please install these in each PC beforehand.

<br>

- Install **Unity 3D** (I used version 2021) <b> You will need to install and Register Unity 3D Software beforehand. 'Unity Hub' is recommended.</b>
- Install **git** software (https://git-scm.com/downloads) for importing Unity asset.
- Clone **this repository**, and Open the project. The first time, some assets downloads would be performed.

<br>
- (Optional) If you want to build Android App, Android build Module (via Unity Hub Setting) / Android SDK is required.

<br>

- Unity 3D Software
> Windows - (https://unity3d.com/get-unity/download)

> Ubuntu - (https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux)



<br>
<br>

### Network
These client have to connect the same network.

# Usage

## 1. Mini Pupper

Open the terminal, Enter this. (Tab No.1)

```
# check your IP address.
ifconfig

cd ~/catkin_ws
source devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch     # with roscore
```

![image](https://user-images.githubusercontent.com/52738228/193747296-7212c0dd-ca4c-4de0-b39f-cd3bc9db8c87.png)

And open new terminal  (Tab No.2) (Ctrl-T key)

```
source devel/setup.bash

# If you try CmdVel sample project
roslaunch mini_pupper bringup.launch

# Or, If you try JointTrajectory sample project,,
roslaunch servo_interface servo_interface.launch

# (If use mini_pupper_ros - ros1 branch (updated Sep, 2022),  )
# roslaunch mini_pupper_bringup bringup.launch
# roslaunch mini_pupper_control servo_interface.launch

```




## 2. PC

Open the Unity project. And open the scene file. 

(Additional packages will start to download automatically.)

Two scenes can be selected.

### Assets/Scenes/MiniPupperCmdVelPose.unity        :    Example for CmdVel message.

* This example shows two virtual joystick.

* When connected to ROS, you can move Mini Pupper with CmdVel messages.

### Assets/Scenes/MiniPupperJointTrajectory.unity   :    Example for JointTrajectory message.  with IK foot positioning.

* This example shows 4 foot position box and a body position box.

* When connected to ROS, you can move Mini Pupper with JointTrajectory messages.

* And the Leg move with IK (forward to foot position box.)


### Setup the ROS network description.

Enter the Mini Pupper's IP address in the "ROS IP Address" fields.

![image](https://user-images.githubusercontent.com/52738228/193615670-20c2eba7-0060-4d01-9398-495fe0ad8d83.png)

![image](https://user-images.githubusercontent.com/52738228/193615722-dc5099be-51d2-41e7-b18d-72bde794342d.png)

So click the play button, then Mini Pupper and Unity were connected.


# Note

This page will be update.

# Author

@devemin

https://twitter.com/devemin

# Reference

https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration

# Licence

MIT License


