import rospy
from std_msgs.msg import Float32,Int32

def talker():
    pubact = rospy.Publisher("/activation_x", Int32, queue_size=1)
    pubvel = rospy.Publisher("/vel_in_x", Float32, queue_size=1)
    rospy.init_node('tester', anonymous=True)
    pubact.publsih(1)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        vel = 0.2
        pubvel.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass