#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
import arbotix_msgs.msg

pub_unity = rospy.Publisher('/unity', String, queue_size=10)
rospy.sleep(0.5)
pub_robot = rospy.Publisher('robot', arbotix_msgs.msg.RobotCommand)
rospy.sleep(0.5)

class Processor(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['no_command','exit_processor','command_for_unity'],
                             input_keys=['tangible','emotion'],
                             output_keys=['unity_command','robot_command'])
        self.data = '00'

    def execute(self, userdata):
        rospy.sleep(0.2)
        rospy.loginfo('Executing state Processor')
        print userdata.tangible
        if self.data != userdata.tangible:
            userdata.unity_command = userdata.tangible
            userdata.robot_command = '2nod'
            self.data = userdata.tangible
            return 'command_for_unity'
        else:
            userdata.robot_command = '0blink'
            return 'no_command'

class UnityPub(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeed','exit_unitypub'],
                             input_keys=['unity_command_pub'])

    def execute(self, userdata):
        msg = String()
        msg.data = userdata.unity_command_pub
        rospy.sleep(0.2)
        pub_unity.publish(msg)
        return 'succeed'
#-----------------------------------------------------------------   
class RobotCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeed','exit_robotcommand'],
                             input_keys=['robot_command_pub'])
        self.data = ''

    def execute(self, userdata):
        s = userdata.robot_command_pub
        if self.data != s:            
            cmd = arbotix_msgs.msg.RobotCommand()
            cmd.priority = int(s[0])
            cmd.command = s[1: ]
            pub_robot.publish(cmd)
            self.data = s
        return 'succeed'
        