#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import  Float32MultiArray

def talker():
    pub = rospy.Publisher("current_position", Float32MultiArray, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        mPosition = Float32MultiArray([0, 0, 0, 1, 1, 1, 1, 0])
        #rospy.loginfo("send_position", "lol")
        pub.publish(mPosition)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
