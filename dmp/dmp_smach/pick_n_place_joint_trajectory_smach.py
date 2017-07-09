#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun dmp dmp_joint_trajectory_action_server.py first!
"""


from dmp.srv import *
import sys
import rospy
from srv_client import *
import srv_action_client
import dmp_joint_trajectory_client
import smach
import smach_ros



class Start_position_to_pick_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing start_position_to_pick_position...')
        global dmp0_traj
        dmp0_traj.parse_file("/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/datasets/go_to_pick_position_dmp.txt")
        rospy.loginfo('read recording file Start_position_to_pick_position trajectory')
        dmp0_traj.start()        
        dmp0_traj.wait()
        rospy.loginfo('succeesfully run start_to_pick_trajectory')    
        return 'Succeed'


        
        
class Pick_position_to_place_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing pick_position_to_place_position...')
        global dmp1_traj
        dmp1_traj.parse_file("/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/datasets/go_to_place_position_dmp.txt")
        dmp1_traj.start()
        dmp1_traj.wait()
        rospy.loginfo('succeesfully run pick_position_to_place_position')
        return 'Succeed'

class  Place_position_to_start_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing place_position_to_start_position..')
        global dmp2_traj
        dmp2_traj.parse_file("/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/datasets/go_back_dmp.txt")
        dmp2_traj.start()
        return 'Succeed'

class Gripper_open(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  Open gripper...')
        global traj
        traj.gripper_open()
        rospy.loginfo("gripper open")        
        return 'Succeed'
class Setup_gripper_statue(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  Open gripper...')
        global traj1
        traj1.gripper_open()
        rospy.loginfo("gripper open")        
        return 'Succeed'

class Gripper_close(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  close gripper...')
        global traj
        traj.gripper_close()
        rospy.loginfo("gripper close")        
        return 'Succeed'

        
def main():
    rospy.init_node("pick_n_place_joint_trajectory")
#    rospy.on_shutdown(shutdown)
    rospy.wait_for_message("/robot/sim/started", Empty) # uncomment this lines in real robot 
    sm = smach.StateMachine(outcomes=['Done'])
    global traj
    global traj1
    global dmp0_traj
    global dmp1_traj
    global dmp2_traj
    traj = srv_action_client.Trajectory("right")
    traj1 = srv_action_client.Trajectory("right")
    dmp0_traj = dmp_joint_trajectory_client.Trajectory()
    dmp1_traj = dmp_joint_trajectory_client.Trajectory()
    dmp2_traj = dmp_joint_trajectory_client.Trajectory()
   
    with sm:
        smach.StateMachine.add('Setup_gripper_statue', Setup_gripper_statue(),
                               transitions={'Succeed':'Start_position_to_pick_position'})
        
        smach.StateMachine.add('Start_position_to_pick_position', Start_position_to_pick_position(),
                               transitions={'Succeed':'Gripper_close'})
        
        smach.StateMachine.add('Gripper_close',Gripper_close(),
                               transitions={'Succeed':'Pick_position_to_place_position'})
                               
        smach.StateMachine.add('Pick_position_to_place_position',Pick_position_to_place_position(),
                               transitions={'Succeed':'Gripper_open'})
        
        smach.StateMachine.add('Gripper_open',Gripper_open(),
                               transitions={'Succeed':'Place_position_to_start_position'})
        
        smach.StateMachine.add('Place_position_to_start_position',Place_position_to_start_position(),
                               transitions={'Succeed':'Done'})
                               
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())


