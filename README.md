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

And Now you can publish to ```/vel_in_x``` , ``` /vel_in_y``` to command linear x and y respectively,```/vel_in_z``` to command angular z,and finally ```/vel_in_alt``` to command the altitude.

The frequency of sending data to the drone is equal to 5Hz.

### Example Moving at constant Velocity:
The ``` move_tester.py ``` file is an example of moving with a constant velocity.

## Using The Interface
To run the other parts of the project, you need to launch the file  ```Sequencer.launch```.

```
$ roslaunch drone_project Sequencer.launch
```
You will get an interface, that will let you choose the mode you want, you can click on Doors, to find opened doors and pass through them, or Hallway to launch the autonomous navigation in the Hallways.

## Customize this project

### Nodes

Each node in this project have a unique name, used to activate it.

They also have the ability to send back confirmation after reaching a target .

Like for example the node responsible of searching for openned doors, it will send a confirmation when a door is detected.

### Create your sequence

In this project, you can run your own sequence, the sequence is a python list that has a specific format.

The Format is the following 

```python
new_seq_list = ["mode_name",[["node_1","node_2"],["cond_1","cond_2"]],...]
self.new_seq = Sequencer(new_seq_list)
```

Where ```mode_name``` is the string that when we receive on the ```/mode``` topic , the sequence will run.
Where ```node_1``` and ```node_2``` are the names of nodes that will be activated in the phase 1 of this sequence, and ```cond_1``` and ```cond_2``` are the names of nodes confirmations we are waiting for to continue to the next step.

### NB: 
Having ```[]``` as condition means always True, and having ```[""]``` means always False. 

### Running the sequence 

To run the sequence, you need to add 2 lines of code to the ```Sequencer.py``` class .

```python
def enter_loop(self,ros_data):
    ####Code
    #if(self.mode == self.new_seq.get_mode()):
        #self.new_seq.seq_fun()
```
Copy and paste these 2 lines, and after that Uncomment them and change the new_seq to the name of the sequence you've defined.

And now, whenever the mode is received in the mode topic, the sequence will run.

And to get sure your sequence will be reset after Init,
```python
def read_mode(self,ros_data):
    self.mode = ros_data.data
    if(self.mode == "init"):
        msg = actMsg()
        msg.node_name = "reset"
        msg.activate = True
        self.actv_publisher.publish(msg)
        self.doors_seq = Sequence(self.doors_seq_list)
        self.hallways_seq = Sequence(self.hallway_seq_list)
        #self.new_seq = Sequence(self.new_seq_list)
    else:
        self.loop_pub.publish(3)
```
Copy the ```self.new_seq = Sequence(self.new_seq_list)``` line , paste it and uncomment int and replace the ```new_seq``` by the name of your sequence.

### Creating your own node

You can also create you own node in this project.

To do so , and if you want your node to be able to work properly in the sequencer, you have to inherit from the class ```NodeActivate```
```python
from activation_class import NodeActivate
class myNewNode(NodeActivate)
    def __init__(self):
        super(myNewNode,self).__init__("newNodeName")
        #Your Code
```
This means that your node has the name ```newNodeName``` and this is the name that will be used to activate it seperatly if you want (Check section)

and to check if the node is activated or not, the variable ```self.activate``` is true when node activated and false otherwise.

As mentionned above, the node may need to send confirmation after reaching the target, to do so, you have to inherit from the class ```returnResp```.

```python
from activation_class import NodeActivate,returnResp
class myNewNode(NodeActivate,returnResp)
    def __init__(self):
        super(myNewNode,self).__init__("newNodeName")
```
And to send the confirmation use the following function,
```python
self.send_conf()
```

## Use project outside sequencer

This project can be used obviously outside the sequencer (You can define your own launch file and run the nodes you want).

But before doing that, there is few things you need to know.

### Activation

To activate a node , you have to publish a specific type of message called ```actMsg``` on the topic ```/activations```.

This message contains 2 fields, a string which is the name of the node you want to activate, the second field is a boolean , which is the activation.

```python
from drone_project.msg import actMsg
import rospy
pub = rospy.Publisher("/activations",actMsg,queue_size=1)
msg = actMsg()
msg.node_name = "nodeToAct" #Name of the node to activate
msg.activate = True #False to deactivate
pub.publish(msg)
```
### Getting Response

As mentionned above also, some nodes send confirmation whenever they reach the target. A node doing this will publish its name to the topic ```/return_resp```.

## Some Useful nodes

### Distance Estimation using camera
This project contains a node able to estimate distances of moving objects using dense optical flow with the "Gunnar Farneback" algorithm.

```nodeName = checkDoors```

To use this module you can just run the ```testDenseOpticalFlow.py``` node and activate it.

To view the histogram showing distances, run the rqt_image_view and choose the topic ```/image_graph```
```
$ rqt_image_view
```
Whenever an opened door is detected, this node will publish to the ```/return_resp``` Topic.

### Vanishing point detection

This project contains a node able to detect the vanishing point in a Hallway using DBSCAN clustering algorithm.

```nodeName = detectVanish```

To use this you need to run the ```image_proc.py``` node and activate it.

This node does not publish on ```/return_resp``` but it will publish the x of this point to the topic ```/centroids```.

### Turn to a specified angle

This project contains a node able to turn the drone to a specific angle.

```nodeName = turnAng```

To use this you need to import the ```TurnAngClass``` class and do like the following exemple :
```python
#!/usr/bin/env python
from TurnAngClass import TurnDrone
      
def main(args):
    rospy.init_node('TurnDrone', anonymous=True)
    sc = TurnDrone(90) ## the angle you want .
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
```
Whenever the turn is done, this node will publish to the ```/return_resp``` Topic.