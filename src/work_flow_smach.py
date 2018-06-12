#!/usr/bin/env python

import rospy
import smach
import smach_ros
import work_flow_states

sm_work_flow = smach.StateMachine(outcomes=['work_flow_shut_down'],
                                  input_keys=['tangible_input','emotion_input'],
                                  output_keys=['robot'])



with sm_work_flow:                                                           
    smach.StateMachine.add('PROCESSOR', 
                           work_flow_states.Processor(),
                           transitions={'no_command':'work_flow_shut_down',
                                        'command_for_unity':'UNITYPUB',
                                        'exit_processor':'work_flow_shut_down'},
                           remapping={'tangible':'tangible_input',
                                      'emotion':'emotion_input',
                                      'unity_command':'unity',
                                      'robot_command':'robot'})
                           
    smach.StateMachine.add('UNITYPUB', 
                           work_flow_states.UnityPub(),
                           transitions={'succeed':'work_flow_shut_down',
                                        'exit_unitypub':'work_flow_shut_down'},
                           remapping={'unity_command_pub':'unity'})