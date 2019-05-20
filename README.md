# Semi-Autonomous Drone Pilot

This ROS package is for a semi-autonomous drone pilot for the Parrot Bebop 2 using image processing (OpenCV) and unsupervised machine learning (Clustering).

## Quick Intro

This project has many parts,The first one is moving the drone at a constant velocity.
The Second part is built on the first one , is used move the drone inside a hallway.
The third part, is used to detect openned doors.

## Use This Package
First Clone this repo in your catkin workspace
```
$ git clone https://gitlab.centralesupelec.fr/obeid_jad/dorne_project.git drone_project
```
Build your workspace
```
$ catkin build
```
## Moving Drone
To do this you need to launch the VelLaunch.launch 

```
$ roslaunch drone_project VelLaunch.launch
```
First start with sending a reset command to this modules by sending any integer to the topics ```/reset_cmd_x```,```/reset_cmd_y``` and ```/reset_cmd_z```.
 
And then publish to ```/vel_in_x``` , ``` /vel_in_y```  and ```/vel_in_z``` the velocities you want the drone to move with.

The frequency of sending data to the drone is equal to 5Hz.

### Example Moving at constant Velocity:
The ``` move_tester.py ``` file is an example of moving with a constant velocity.

## Using The Interface
To run the other parts of the project, you need to launch the file  ```Sequencer.launch```.

```
$ roslaunch drone_project Sequencer.launch
```
You will get an interface, that will let you choose the mode you want, you can click on Doors, to find opened doors and pass through them, or Hallway to launch the autonomous navigation in the Hallways.

## Some Useful Nodes

### Activation and Deactivation

To activate or deactivate a node, you need to send an activation to the topic ```/activations```, The activation command is a ```<nodeName>_1``` to activate and ```<nodeName>_0``` to deactivate.

Note that sending ```reset``` will deactivate all nodes.

### Distance Estimation using camera
This project contains a node able to estimate distances of moving objects using dense optical flow with the "Gunnar Farneback" algorithm.

nodeName = checkDoors

To use this module you can just run the ```testDenseOpticalFlow.py``` node :

```
$ rosrun drone_project testDenseOpticalFlow.py
```
And to view the histogram showing distances, run the rqt_image_view and choose the topic ```/image_graph```
```
$ rqt_image_view
```
Whenever an opened door is detected, this node will publish ```1```, to the topic ```/door_det ```.

### Vanishing point detection

This project contains a node able to detect the vanishing point in a Hallway using DBSCAN clustering algorithm.

nodeName = detectVanish

To use this you need to run the ```image_proc.py``` node :

```
$ rosrun drone_project image_proc.py
```

This node will publish the x of this point to the topic ```/centroids```.

### Turn to a specified angle

This project contains a node able to turn the drone to a specific angle.

nodeName = turnAng

To use this you need to run the ```turnAng.py``` node :

```
$ rosrun drone_project turnAng.py
```

If you run this node directly, the drone will move to 90 degrees.

If not, publish the desired angle in degrees to the topic ```/ang_in```.


