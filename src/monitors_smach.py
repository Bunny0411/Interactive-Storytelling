#!/usr/bin/env python

import rospy
import smach
import smach_ros
import monitors
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def child_term_cb(outcome_map):

  if outcome_map['MONITOR_TANGIBLE'] == 'invalid':
      return True
  else:
      return False

sm_monitor = smach.Concurrence(outcomes=['monitor_continue','monitor_done'],
                               default_outcome='monitor_continue',
                               input_keys=['tangible_output','emotion_output'],
                               output_keys=['tangible_output','emotion_output'],
                               child_termination_cb = child_term_cb)
with sm_monitor:        
    smach.Concurrence.add('MONITOR_TANGIBLE', 
                          smach_ros.MonitorState("/sm_reset",
                                                 String,                    
                                                 monitors.monitor_tangible_cb,
                                                 output_keys=['tangible_output_monitor']),
                          remapping={'tangible_output_monitor':'tangible_output'})

    smach.Concurrence.add('MONITOR_EMOTION', 
                          smach_ros.MonitorState("/sm_emotion",
                                                 Float32MultiArray,                    
                                                 monitors.monitor_emotion_cb,
                                                 output_keys=['emotion_output_monitor']),
                          remapping={'emotion_output_monitor':'emotion_output'})                              
                              
