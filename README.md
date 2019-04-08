# TESTING 
-Testing compatibility between the image processing and sending constant V

# Semi-Autonomous Drone Pilot

This ROS package is for a semi-autonomous drone pilot for the Parrot Bebop 2 using image processing (OpenCV) and unsupervised machine learning (Clustering).

## Quick Intro

This project has 2 parts, the first one is some nodes to ensure the navigation of the drone in constant velocities on the X,Y and Z axis since the Twist command provided by ROS is not a velocity but an acceleration.
The Second part is built on the first one and it some nodes doing image processing and sending necessary data to the other nodes in order to navigate the drone in the middle of A hallway.
## NOTE:
The First part is totaly isolated from the second one and can be used on any Parrot Bebop Drone.
In this File , you can see how to use the first one, the second one is coming soon 

## Using The Constant V Command
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
First start with sending a reset command to this modules by sending any integer to the topics ```reset_cmd_x```,```reset_cmd_y``` and ```reset_cmd_z```.

After that activate the function you want to use , For example, if you want to move over x publish Int32 message ```1``` to the ```/activation_x```.

And then publish to ```/vel_in_x``` , ``` /vel_in_y```  and ```/vel_in_z``` the velocities you want the drone to move with.
Note that the rate of sending data to the drone is equal to the fastest rate between the 3 ``` /vel_in ``` topics.

### Example:
The ``` vel_tester.py ``` file is an example on moving with a constant velocity.

## Indoor navigation
### Hallways navigation
Soon ...
### Distance Estimation using camera
Soon ...

