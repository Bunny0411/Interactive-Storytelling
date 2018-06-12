#!/usr/bin/env python

import rospy
import smach
import smach_ros
import monitors
from std_msgs.msg import String
import robot_states
from arbotix_msgs.msg import RobotCommand
import time

system_robot_wrapper = smach.StateMachine(outcomes=['system_robot_shut_down'])

server1 = robot_states.Server('test_action1',0)
server2 = robot_states.Server('test_action2',1)
server5 = robot_states.Server('test_action5',4)
server6 = robot_states.Server('test_action6',5)
with system_robot_wrapper:
    
    def child_term_cb(outcome_map):
      if outcome_map['MONITOR_ROBOT'] == 'invalid':
          return True
      else:
          return False
    
    sm_system_robot = smach.Concurrence(outcomes=['system_robot_done','system_robot_continue'],
                                  default_outcome='system_robot_continue',
                                  child_termination_cb = child_term_cb)
    
    sm_system_robot.userdata.robot_command = [0,'blink']
    
    with sm_system_robot:

        smach.Concurrence.add('MONITOR_ROBOT',
                              smach_ros.MonitorState("/robot",
                                                     RobotCommand,
                                                     monitors.monitor_robot_command,
                                                     input_keys=['current_command'],
                                                     output_keys=['robot_cmd_output']),
                              remapping={'robot_cmd_output':'robot_command',
                                         'current_command':'robot_command'})

        smach.Concurrence.add('ROBOT_CONTROL',
                              robot_states.sm_robot_control,
                              remapping={'robot_cmd_input':'robot_command'})
                              
    smach.StateMachine.add('SYSTEM_ROBOT',
                           sm_system_robot,
                           transitions={'system_robot_continue':'SYSTEM_ROBOT',
                                        'system_robot_done':'system_robot_shut_down'})
                                  

