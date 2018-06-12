#!/usr/bin/env python

import rospy
import smach
import smach_ros
import arbotix_msgs
import arbotix_python


def monitor_tangible_cb(ud, msg):

    ud.tangible_output_monitor = msg.data
 
    return False

def monitor_emotion_cb(ud, msg):
    #print ud.tangible_output
    ud.emotion_output_monitor = msg.data
    return False

def monitor_robot_command(ud, msg):
    #print ud.tangible_output
    if ud.current_command[0] < msg.priority:
        ud.robot_cmd_output = [msg.priority, msg.command]
        return False
    else:
        return True



'''
def main():
    rospy.init_node("monitor_example")
    
    sm = smach.StateMachine(outcomes=['DONE'])
    sm.userdata.sm_output = 0
    with sm:
        smach.StateMachine.add('FOO', smach_ros.MonitorState("/sm_reset",Int32, monitor_tangible_cb,input_keys=['tangible_input'],output_keys=['tangible_output']),remapping={'tangible_input':'sm_output','tangible_output':'sm_output'}, transitions={'invalid':'DONE', 'valid':'FOO', 'preempted':'FOO'})


    sm.execute()
    rospy.spin()

if __name__=="__main__":
    main()
'''