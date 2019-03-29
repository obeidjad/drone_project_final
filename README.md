# Semi-Autonomous Drone Pilot

This ROS package is for a semi-autonomous drone pilot for the Parrot Bebop 2 using image processing (OpenCV) and unsupervised machine learning (Clustering).

## Quick Intro

This project has 2 parts, the first one is some nodes to ensure the navigation of the drone in constant velocities on the X,Y and Z axis since the Twist command provided by ROS is not a velocity but an acceleration.
The Second part is built on the first one and it some nodes doing image processing and sending necessary data to the other nodes in order to navigate the drone in the middle of A hallway.
##### NOTE:
The First part is totaly isolated from the second one and can be used on any Parrot Bebop Drone.
## Using The Constant V Command
To use this feature you need to launch the Vellaunch.launch 

```
$ roslaunch drone_project Vellaunch.launch
```
After that activate the function you want to use , For example, if you want to move over x send 1 to the /activation_x
And then publish to ```/vel_in_x``` and ``` /vel_in_y```  and ```/vel_in_z``` the velocities you want the drone to move with.
