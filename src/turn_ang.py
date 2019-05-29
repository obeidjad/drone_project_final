#!/usr/bin/env python
from TurnAngClass import TurnDrone
      
def main(args):
    rospy.init_node('TurnDrone', anonymous=True)
    sc = TurnDrone(90)
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

