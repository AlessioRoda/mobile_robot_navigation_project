# Software Architecture for Robotics Assignment
## Authors: Girardo Ermanno S4506472, Roda Alessio S4458313, Petrocco Enzo Ubaldo S4530363
## Date: 15/06/2021

## General requirements of the project: 

The aim of this repository is:
   * allow localization and path planning of a non holonomic Robot in the unknown environment
   * mapping the area while it moves
   * a physics realization on a autonomous vehicle made by Husqvarna company

At start a new goal is generated, the robot will independently reach the target while scanning the surrounding environment, avoiding the walls.

## Components used to realize the project:

The Husqvarna Automover equipped of:
   * LIDAR sensor in order to see the walls and obstacles in the environment
   * Actuators and two wheels in order to move
    
   ![Husqvarna_Automover](https://user-images.githubusercontent.com/48509825/120933380-3350f780-c6fa-11eb-8283-4c6349912027.jpg)
    
The ROS navigation pkg move base. It is an action in order to set a goal in the world and via a planner it guides the robot along an optimized path.
For further information please check the Wiki: http://wiki.ros.org/move_base

The ROS mapping pkg slam_gmapping. It has the aim of generating a map of the enviroment. It localizes the robot respect to the world and scans the environment via laser sensor. For further information please check the Wiki: http://wiki.ros.org/gmapping

The following UML represent the component description of the architecture and the temporal behaviour of it.

### UML Component Diagram:

![Component_Diagram](https://user-images.githubusercontent.com/48509825/121780122-3ab34d80-cb9f-11eb-8466-50d92285b05c.png)



### UML Temporal Diagram:

  ![MicrosoftTeams-image (1)](https://user-images.githubusercontent.com/48509825/120934101-43b6a180-c6fd-11eb-8dc7-d30350df8cd8.png)


## About the simulation:

The scene is developed in a cross-platform game engine Unity developed by Unity Technologies.
In order to download it click on the following link and install the 2020.2.2 version
https://unity3d.com/get-unity/download/archive

Once Unity is installed you can find all the source code to establish the connection with the Robot Operating System here: https://github.com/TheEngineRoom-UniGe/SofAR-Mobile-Robot-Navigation

The robot in the Unity scene is a Husky autonomous vehicle: https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/


  ![husky-a200-ugv-mobile-base](https://user-images.githubusercontent.com/48509825/120934828-34852300-c700-11eb-97cc-2817a6503652.jpg)
  
Unity will publish the position of the robot on /odometry_frame topic and the data acquired by the LIDAR sensor via /laser_scan topic.
On ROS, a node odometry_publisher.py, transforms the data on /odometry_frame topic to /odom topic. In this way move base is able to receive the position and the laser sensor data. It will publish the command velocity on the /cmd_vel topic in order to guide the Robot.

### Comunication between ROS and Unity

First of all you must establish the connection between ROS and Unity.
You can do this in multiple ways via:
   * Two machines, of course one with the Unity simulation and the other one with the ROS noetic version.
     In this case the machines have to be in the same LAN or you can create a virtual LAN, for instance downloading a proper software (eg Hamachi https://www.vpn.net/ )
   * On the same machine via Virtual Machine or Docker container.

The IP adresses of the two machines must be specified: find params.yaml into config folder in the ROS pkg and into the Robotics window of the Unity simulation.
Make sure to disable the Firewall on the machine that runs Unity.
Once you have done this steps try to digit into ROS_ws:
```
roslaunch mobile_robot_navigation_project navigation.launch
```
Start the simulation on Unity, if you receive the handshake msg on ROS the connection is successfully done.

### Husky robot for the simulation and libraries
In order to have the Husky robot description you need to download the Husky URDF, please click and download the following pkg and make it on your ros_ws https://github.com/husky/husky
Once downloaded run into your machine:
```
sudo apt-get install ros-noetic-lms1xx
```
On your ROS machine check to have installed the following libraries:
The navigation package, in order to use move_base planner.
 ```
 sudo apt-get install ros-noetic-navigation 
 ```
 The open SLAM Gmapping package: 
 ```
 sudo apt-get install ros-noetic-openslam-gmapping
 ```
 
 In order to use the gmapping package you must install and make the slam_gmapping pkg into your ROS workspace.
 To do this step please move into your src folder and run the following command:
 ```
 git clone https://github.com/ros-perception/slam_gmapping
``` 
### Start the simulation:

In order to run the simulation do the following commands:

```
  roslaunch mobile_robot_navigation_project gmapping.launch 
```
This launch file will execute the slam_gmapping node setting all the parameters needed to publish a map.
![rviz_screenshot_2021_06_06-17_21_52](https://user-images.githubusercontent.com/48509825/120936234-2686d080-c707-11eb-904d-de64e9cd156f.png)

```
roslaunch mobile_robot_navigation_project move_base2.launch
```
This launch file will execute the move_base node setting all the parameters in order to set the planner,the coastmap and the general move base parameters.
You can find all the parameters in the folder /param.
```
roslaunch mobile_robot_navigation_project view_model.launch
```
This launch file will load the URDF of the Husky robot and all the transformation needed.
It execute also a configuration file for RViz showing three map, the Husky robot and the path of the planner. The fixed frame is setted to /odom
   * one map suscribed to /map
   * one map subscribed to /local_coastmap
   * one map subscribed to /global_coastmap
Once executed this launch file, you should see the RViz emulator with a pair of error. Don't worry when you will launch the simulation on Unity these will disappear.
   
```
roslaunch mobile_robot_navigation_project navigation.launch
```
This launch file open the communication on port 10000.
Now you can start the simulation on Unity side.
Press the button Move! in order to send the goal, the robot should start to move and reaching the goal while the map on RViz should be update.

Finally if you run 
```
rosrun rqt_graph rqt_graph
```
you should see a behaviour like that
  ![rosgraphSofar](https://user-images.githubusercontent.com/48509825/120936219-07883e80-c707-11eb-8c79-d3fde2a12c06.png)
