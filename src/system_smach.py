#!/usr/bin/env python

import rospy

rospy.init_node('smach_example_state_machine', anonymous=True)

import smach
import smach_ros
import monitors_smach
import work_flow_smach
import work_flow_states
import robot_smach

        
def main():

    
    
    system = smach.Concurrence(outcomes=['finished'],
                               default_outcome='finished')
    
    with system:
        
        
        sm_system_wrapper = smach.StateMachine(outcomes=['system_shut_down'])
        
        with sm_system_wrapper:
            
            sm_system = smach.Concurrence(outcomes=['system_done','system_continue'],
                                          default_outcome='system_continue',
                                          outcome_map={'system_done':
                                                           {'SYSTEM_MONITOR':'monitor_done',
                                                            'SYSTEM_WORKFLOW':'work_flow_shut_down',
                                                            'ROBOT_COMMAND_PROCESSOR':'exit_robotcommand'}})
            sm_system.userdata.sm_tangible = '00'
            sm_system.userdata.sm_emotion = [0,0,0]
            sm_system.userdata.sm_robot = '0 '
            with sm_system:
                smach.Concurrence.add('SYSTEM_MONITOR', 
                                      monitors_smach.sm_monitor,
                                      remapping = {'tangible_output':'sm_tangible',
                                                   'emotion_output':'sm_emotion'})
            
                smach.Concurrence.add('SYSTEM_WORKFLOW',
                                      work_flow_smach.sm_work_flow,
                                      remapping = {'tangible_input':'sm_tangible',
                                                   'emotion_input':'sm_emotion',
                                                   'robot':'sm_robot'})
                                  
                smach.Concurrence.add('ROBOT_COMMAND_PROCESSOR',
                                      work_flow_states.RobotCommand(),
                                      remapping = {'robot_command_pub':'sm_robot'})
            
            smach.StateMachine.add('SYSTEM',
                                   sm_system,
                                   transitions={'system_done':'system_shut_down',
                                                'system_continue':'SYSTEM'})
            
        smach.Concurrence.add('SYSTEM_WRAPPER',
                              sm_system_wrapper)
        smach.Concurrence.add('SYSTEM_ROBOT_WRAPPER',
                              robot_smach.system_robot_wrapper) 
            
    sis = smach_ros.IntrospectionServer('server_name', system, '/SM_ROOT')
    sis.start()
    outcome = system.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()