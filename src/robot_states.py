#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from arbotix_msgs.msg import *
from actionlib import *
from actionlib_msgs.msg import *
from arbotix_msgs.srv import SetSpeed
from arbotix_python.joints import *
from arbotix_python.servo_controller import *
from sensor_msgs.msg import JointState
import robot_commands
import time
import arbotix_msgs.msg

class Server:
    def __init__(self,name,i):
        pubs = ['/servo1_joint/command','/servo2_joint/command','/servo3_joint/command','/servo4_joint/command','/servo5_joint/command','/servo6_joint/command']
        self.joints = ['servo1_joint','servo2_joint','servo3_joint','servo4_joint','servo5_joint','servo6_joint']
        self.services = ['/servo1_joint/set_speed','/servo2_joint/set_speed','/servo3_joint/set_speed','/servo4_joint/set_speed','/servo5_joint/set_speed','/servo6_joint/set_speed']
        default_speed = [0.6, 0.6, 0.3, 0.3, 0.4, 0.3]
        

        self.stop = False
        self._sas = SimpleActionServer(name,
                ControlAction,
                execute_cb=self.execute_cb)
        self.pub = rospy.Publisher(pubs[i], Float64, queue_size=10)
        self.pose = 0.0
        self.sub = rospy.Subscriber('/joint_states', JointState, self.stateCb)
        self.i = i
        self.default_speed = default_speed[i]
        
    def stateCb(self, msg):
        idx = msg.name.index(self.joints[self.i])
        self.pose = msg.position[idx]


    def SetSpeed_client(self, speed):
         servo = self.services[self.i]
         rospy.wait_for_service(servo)
         
         try:
             set_speed = rospy.ServiceProxy(servo,SetSpeed)
             s = set_speed(speed)
             return s
         except rospy.ServiceException, e:
             print ("Service call failed") 


    def execute_cb(self, msg):

        targets = list(msg.target)
        
        if len(targets) == 0:
            self._sas.set_succeeded()
        else:
            for t in targets:
                self.SetSpeed_client(self.default_speed)
                
                if abs(t - self.pose) > self.default_speed:
                    speed = (abs(self.pose - t) - 0.5 * self.default_speed)
                else:
                    speed = self.default_speed
                    
                self.pub.publish(t)
                
                if self._sas.is_preempt_requested():
                    self._sas.set_preempted()
                    self.stop = True
                    break
                    
                while not rospy.is_shutdown():
                    
                    if abs(self.pose - t) > 0.1:
                        self.SetSpeed_client(speed)
                    else:
                        self.SetSpeed_client(self.default_speed)
                        
                    self.pub.publish(t)
                    
                    if abs(t - self.pose) < 0.02:
                        break
            if not self.stop:
                self._sas.set_succeeded()
            else:
                self.stop = False


      
sm_robot_control = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                        default_outcome='succeeded',
                        input_keys = ['robot_cmd_input'])

with sm_robot_control:

    def goal_callback1(userdata,default_goal):
        userdata.cmd1[0] = 0
        goal = ControlGoal()        
        goal.target = robot_commands.str2command(userdata.cmd1[1], 1)
        return goal
    
    def goal_callback2(userdata,default_goal):
        goal = ControlGoal()        
        goal.target = robot_commands.str2command(userdata.cmd2[1], 2)
        return goal

    def goal_callback5(userdata,default_goal):
        goal = ControlGoal()        
        goal.target = robot_commands.str2command(userdata.cmd5[1], 5)
        return goal
    
    def goal_callback6(userdata,default_goal):
        goal = ControlGoal()        
        goal.target = robot_commands.str2command(userdata.cmd6[1], 6)
        return goal
    
    smach.Concurrence.add('GOAL_CB1',
                          smach_ros.SimpleActionState('test_action1', ControlAction,
                                                      goal_cb = goal_callback1,
                                                      input_keys = ['cmd1'],
                                                      output_keys = ['cmd1']),
                          remapping={'cmd1':'robot_cmd_input'})
    '''                         
    smach.Concurrence.add('GOAL_CB2',
                          smach_ros.SimpleActionState('test_action2', ControlAction,
                                                      goal_cb = goal_callback2,
                                                      input_keys = ['cmd2']),
                          remapping={'cmd2':'robot_cmd_input'})
    '''
    smach.Concurrence.add('GOAL_CB5',
                          smach_ros.SimpleActionState('test_action5', ControlAction,
                                                      goal_cb = goal_callback5,
                                                      input_keys = ['cmd5']),
                          remapping={'cmd5':'robot_cmd_input'})

    smach.Concurrence.add('GOAL_CB6',
                          smach_ros.SimpleActionState('test_action6', ControlAction,
                                                      goal_cb = goal_callback6,
                                                      input_keys = ['cmd6']),
                          remapping={'cmd6':'robot_cmd_input'})
'''
 
def main():
    rospy.init_node('smach_example_actionlib')

    # Start an action server
    server1 = Server('test_action1',0)
    server2 = Server('test_action2',1)
    server5 = Server('test_action5',4)

    # Create a SMACH state machine
    sm0 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                            default_outcome='succeeded')

    # Open the container
    with sm0:

        def goal_callback1(userdata, default_goal):
            goal = ControlGoal()
            goal.target = [-0.45,-0.25, -0.95, -0.25]
            return goal

        def goal_callback2(userdata, default_goal):
            goal = ControlGoal()
            goal.target = [-0.3,-0.1,-0.3,-0.1]
            return goal


        smach.Concurrence.add('GOAL_CB1',
                               smach_ros.SimpleActionState('test_action1', ControlAction,
                                                       goal_cb = goal_callback1))
                               
        smach.Concurrence.add('GOAL_CB2',
                               smach_ros.SimpleActionState('test_action2', ControlAction,
                                                       goal_cb = goal_callback1))
                               
        smach.Concurrence.add('GOAL_CB5',
                               smach_ros.SimpleActionState('test_action5', ControlAction,
                                                       goal_cb = goal_callback2))
        # For more examples on how to set goals and process results, see 
        # executive_smach/smach_ros/tests/smach_actionlib.py

    # Execute SMACH plan
    outcome = sm0.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
'''
                 
                    
                
        
        
        
        
        

