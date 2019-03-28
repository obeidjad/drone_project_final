# Semi-Autonomous Drone Pilot

This ROS package is for a semi-autonomous drone pilot for the Parrot Bebop using image processing (OpenCV) and unsupervised machine learning (Clustering).

## Quick Intro

This project has 2 parts, the first one is some nodes to ensure the navigation of the drone in constant velocities on the X,Y and Z axis since the Twist command provided by ROS is not a velocity but an acceleration.
The Second part is built on the first one and it some nodes doing image processing and sending necessary data to the other nodes in order to navigate the drone in the middle of A hallway.
##### NOTE:
The First part is totaly isolated from the second one and can be used on any Parrot Bebop Drone.
#### To be Continued ..