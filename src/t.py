#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from arbotix_msgs.msg import RobotCommand


def talker():
    pub = rospy.Publisher('custom_chatter', RobotCommand)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = RobotCommand()
    msg.priority = 1
    msg.command = "hh"

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
