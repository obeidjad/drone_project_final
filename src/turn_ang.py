#!/usr/bin/env python
from TurnAngClass import TurnDrone
import sys
import rospy
def main(args):
    rospy.init_node('TurnDrone2', anonymous=True)
    sc = TurnDrone(90)
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)