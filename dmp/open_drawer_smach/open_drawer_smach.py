#!/usr/bin/env python

from dmp.srv import *
import sys
import rospy
from srv_client import *
import dmp_r_joint_trajectory_client
import smach
import smach_ros
import config



class Go_to_start_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Go_to_start_position...')
        global dmp0_traj
        dmp0_traj.parse_file(config.go_to_start_position_path)
        
        rospy.loginfo('read recording file Go_to_start_position trajectory')
        dmp0_traj.start()        
        dmp0_traj.wait()
        rospy.loginfo('succeesfully run start_to_pick_trajectory')    
        return 'Succeed'
        
        
class Go_to_gripper_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Go_to_gripper_position...')
        global dmp1_traj       
        dmp0_traj.parse_file(config.go_to_gripper_position_path)
        
        rospy.loginfo('read recording file Go_to_gripper_position trajectory')
        dmp1_traj.start()        
        dmp1_traj.wait()
        rospy.loginfo('succeesfully run Go_to_gripper_position_trajectory')    
        return 'Succeed'
        
class Go_back(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Go_back...')
        global dmp2_traj       
        dmp0_traj.parse_file(config.go_back_path)
        
        rospy.loginfo('read recording file Go_back trajectory')
        dmp2_traj.start()        
        dmp2_traj.wait()
  
        rospy.loginfo('succeesfully run Go_back_trajectory')    
        return 'Succeed'

class Pick_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Pick_object...')
        global dmp2_traj       
        dmp0_traj.parse_file("")
        
        rospy.loginfo('read recording file Pick_object trajectory')
        dmp2_traj.start()        
        dmp2_traj.wait()

        rospy.loginfo('succeesfully run Pick_object_trajectory')    
        return 'Succeed'

class Go_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Go_forward...')
        global dmp3_traj       
        dmp0_traj.parse_file(config.go_forward_path)        
        rospy.loginfo('read recording file Go_forward trajectory')
        dmp3_traj.start()        
        dmp3_traj.wait()

        rospy.loginfo('succeesfully run Go_forward_trajectory')    
        return 'Succeed'

class Gripper_open(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  Open gripper...')
        global dmp1_traj
        dmp1_traj.gripper_open()
        rospy.loginfo("gripper open")        
        return 'Succeed'

class Gripper_close(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  close gripper...')
        global dmp1_traj
        dmp1_traj.gripper_close()
        rospy.loginfo("gripper close")        
        return 'Succeed'

def main():
    rospy.init_node("open_drawer_trajectory")
#    rospy.on_shutdown(shutdown)
#    rospy.wait_for_message("/robot/sim/started", Empty) # uncomment this lines in real robot 
    sm = smach.StateMachine(outcomes=['Done'])
    global dmp0_traj
    global dmp1_traj
    global dmp2_traj
    global dmp3_traj
    dmp0_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp1_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp2_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp3_traj = dmp_r_joint_trajectory_client.Trajectory()
   
    with sm:
#        smach.StateMachine.add('Setup_gripper_statue', Setup_gripper_statue(),
#                               transitions={'Succeed':'Start_position_to_pick_position'})
        
        smach.StateMachine.add('Go_to_start_position', Go_to_start_position(),
                               transitions={'Succeed':'Go_to_gripper_position'})
        
        smach.StateMachine.add('Go_to_gripper_position',Go_to_gripper_position(),
                               transitions={'Succeed':'Gripper_close'})
                               
        smach.StateMachine.add('Gripper_close',Gripper_close(),
                               transitions={'Succeed':'Go_back'})
        
        smach.StateMachine.add('Go_back',Go_back(),
                               transitions={'Succeed':'Gripper_open'})
        
        smach.StateMachine.add('Gripper_open',Gripper_open(),
                               transitions={'Succeed':'Done'})
       
#        smach.StateMachine.add('Go_forward',Go_forward(),
#                               transitions={'Succeed':'Done'})                       
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    rospy.spin()
