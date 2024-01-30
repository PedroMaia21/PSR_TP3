# PSR_TP3 - Robutler 1
Repository to the TP3 group project for the PSR 2023/2024 classes.

<img src="https://github.com/PedroMaia21/PSR_TP3/assets/116599734/3f149036-bacc-4832-b6a0-818f55ece454" width="150" height="150"/> ![UA_logo](https://github.com/PedroMaia21/PSR_TP3/assets/116599734/b4293add-d23f-4678-a21c-d78dd736d7ef) 



## Project Description
This project represents the creation of a robutler - robot to do easy domestic chores based in the [turtlebot3 waffle_pi](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) - and its testing in a simulated gazebo world.

This robot is capable of moving to any place in the house and identify objects as commanded

The tecnologies used to make this project were:
- ROS
- Gazebo and Rviz packages
- Moveit package
- [yolov7](https://github.com/lukazso/yolov7-ros)

The challenges of this project consisted in learning how to understand and sucessfully use the ROS environment as well as the development of advance programming knowledge, like the usage of artificial vision to execute object recognition, the commanding and manipulation of robots and robotic manipulators as well as the understanding of node communication.

Something that still needs development is the control of the robotic arm, challenge that was way to complex to trully understand at this point in time.

## Installing and running the project
#### Installing

To execute the instalation of this project, it is necessary to have the ROS workspace operational ([instalation guide here](http://wiki.ros.org/noetic/Installation))
- Note that this project is based in ROS noetic and may have comapatibility issues with other ROS distros

Other dependencies may include:
- turtlebot3 package
- robot_state_publisher package
- joint_state_publisher package
- Rviz package
- gazebo_ros package
- moveit package
- other python libraries

After the cloning to the repository to the source file in you ROS workspace you should compile the packages using:
```
catkin buld
source devel/setup.bash
```
or
```
catkin_make
```

After this steps, the programm is ready to run

#### Running

To run this project, all you have to do is write in your terminal
```
roslaunch robutler1_bringup main.launch
```
or, if you have a low spec computer and don't need the spawning of the gazebo GUI, you can run
```
roslaunch robutler1_bringup main_test_mode.launch
```
After this command will appear a Rviz window and a Robutler Simulation Control window as can be seen in the following image:

![Screenshot after launching the main_test_mode.launch file](https://github.com/PedroMaia21/PSR_TP3/assets/116599734/062eaccd-c616-47c7-b269-13cd6f870825)

There is also an alternative way (not recomended) to launch every file one by one (and consequantly in different terminals) and this may be a good way to troubleshoot if some error occurs during the launch of the aplication

The commands are (remember that each line should be in a new terminal window):
```
roslaunch robutler1_bringup gazebo.launch #or roslaunch robutler1_bringup gazebo_test_mode.launch
roslaunch robutler1_navigation localization.launch
roslaunch robutler1_bringup bringup.launch
roslaunch robutler1_navigation navigation.launch
roslaunch robutler1_detection detection.launch
rosrun robutler1_interface mission_manager.py
roslaunch robutler1_interface menu.launch
```

## Rviz Interface explained

As is possible to see in the next image the visual interface in the Rviz is composed by 3 camera images and a map in the center with the robot model

The map was obtained using the slam method and is stored in the package robutler1_navigation in the folder /maps

The 3 camara images consist in:
- a fixed camera in the base of the robot (botom right corner)
- a moving camera in the gripper of the robutler arm (top right corner)
- and a yolo image with the detections (bottom left corner) - This camera only becomes active when its needed to look for some object

![Screenshot from 2024-01-30 02-36-50](https://github.com/PedroMaia21/PSR_TP3/assets/116599734/baaee741-0753-463e-90e5-19d0ebee9cf8)

## How to controll the robot

The control of the robutler can be done in 3 different ways:
- Using the Robutler Simulation Control panel to directly send commands to the robot (Warning: to use the 'Spawn object', 'Move to location' and 'Search for object in location' buttons, should only be pressed when one object and one location option had been selected)(other note: the quantity input box only affects the behaviour of the 'Spawn object' button)
- Using the interactive marker above the robutler, by pressing it with the right mouse button
- Be calling the scripts individually (not recomended)

## Functionalities/Improvements

- [x] Robot movimentation:
    - [x] using manual mode (teleop)
    - [x] using semantic information eg:"Kitchen"
    - [x] using autonomous searching
- [x] Perception:
    - [x] find color spheres
    - [x] find objects
    - [x] find persons
    - [ ] Count different objets(with same properties)
- [x] Objects:
    - [x] spawn objects in determined location
    - [x] spawn specified object at random place in the house

## Missions

- [x] Move to specified rooms
- [x] Robot search for spheres in selected room/everywhere
- [x] Robot search for objects in selected room/everywhere
- [x] Robot search for someone in selected room/everywhere
- [ ] Robot verify if table is cleaned or not
- [x] Robot photograph selected room
- [x] Robot search for someone in the house
- [ ] Robot count the number of blue boxes in the appartment
- [ ] Robot touch objects.
- [ ] Robot move objects (75%) 
 
 ## Credits
 
The members involved in this project are: Maria Rodrigues ¹, Pedro Maia ², Salomé Dias ³

¹ 102384, rodrigues.mariaalves@ua.pt

² 102959, pedromaia@ua.pt

³ 118163, salome.marie@ua.pt
