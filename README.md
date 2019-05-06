# TESTING 
-Testing disable activation (Done, Activation will be removed)

-Testing detecting doors using optical flow (Done, works very well !!)

-Testing the Turning to a specified angle (Done, it works)

# Semi-Autonomous Drone Pilot

This ROS package is for a semi-autonomous drone pilot for the Parrot Bebop 2 using image processing (OpenCV) and unsupervised machine learning (Clustering).

## Quick Intro

This project has 2 parts, the first one is some nodes to ensure the navigation of the drone in constant velocities on the X,Y and Z axis since the Twist command provided by ROS is not a velocity but an acceleration.
The Second part is built on the first one and it some nodes doing image processing and sending necessary data to the other nodes in order to navigate the drone in the middle of A hallway.
## NOTE:
The First part is totaly isolated from the second one and can be used on any Parrot Bebop Drone.
In this File , you can see how to use the first one, the second one is coming soon 

## Moving The Drone
First Clone this repo in your catkin workspace
```
$ git clone https://gitlab.centralesupelec.fr/obeid_jad/dorne_project.git drone_project
```
Build your workspace
```
$ catkin build
```
To use this feature you need to launch the VelLaunch.launch 

```
$ roslaunch drone_project VelLaunch.launch
```
First start with sending a reset command to this modules by sending any integer to the topics ```/reset_cmd_x```,```/reset_cmd_y``` and ```/reset_cmd_z```.

(Recommanded) 

Wait for receiving the acknoledge from the drone , on the topics ```/ack_res_x``` , ```/ack_res_y``` and ```/ack_res_z```. Once the reset is done you will receive ```1``` on these 3 topics.

And then publish to ```/vel_in_x``` , ``` /vel_in_y```  and ```/vel_in_z``` the velocities you want the drone to move with.

Note that the rate of sending data to the drone is equal to the fastest rate between the 3 ``` /vel_in ``` topics.

### Example:
The ``` move_tester.py ``` file is an example of moving with a constant velocity.
The ``` turn_to_angle.py ``` file is an example of defining a specific angle and moving the drone to this angle. 

## Indoor navigation
### Hallways navigation
Soon ...
### Distance Estimation using camera
This project contains a node able to estimate distances of moving objects using dense optical flow with the "Gunnar Farneback" algorithm.

To use this module you can just run the ```testDenseOpticalFlow.py``` node :

```
$ rosrun drone_project testDenseOpticalFlow.py
```

and to view the histogram showing distances, run the rqt_image_view and choose the topic ```/image_graph```
```
$ rqt_image_view
```

This node will print out in the terminal whenever an opened door is detected and it will specify where is the door on the image