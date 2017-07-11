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
import dmp_r_joint_trajectory_client
import smach
import smach_ros
import config
from std_msgs.msg import Empty



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
        dmp0_traj.stop()
        rospy.loginfo('succeesfully run start_to_pick_trajectory')    
        return 'Succeed'


        
        
class Go_to_gripper_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing Go_to_gripper_position......')
        global dmp1_traj
        dmp1_traj.parse_file(config.go_to_gripper_position_path)
        dmp1_traj.start()
        dmp1_traj.wait()
        rospy.loginfo('succeesfully run Go_to_gripper_position_trajectory')
        return 'Succeed'

class Go_back(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go_back..')
        global dmp2_traj
        dmp2_traj.parse_file(config.go_back_path)
        dmp2_traj.start()
        dmp2_traj.wait()
        return 'Succeed'

class Go_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go_forward..')
        global dmp3_traj
        dmp3_traj.parse_file(config.go_forward_path)
        dmp3_traj.start()
        dmp3_traj.wait()
        return 'Succeed'
class Go_back_to_start_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go_back_to_start_position...')
        global dmp4_traj
        dmp4_traj.parse_file(config.go_back_to_start_position_path)
        dmp4_traj.start()
        dmp4_traj.wait()       
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
    rospy.init_node("pick_n_place_joint_trajectory")
#    rospy.on_shutdown(shutdown)
#    rospy.wait_for_message("/robot/sim/started", Empty) # uncomment this lines in real robot 
    sm = smach.StateMachine(outcomes=['Done'])
    global dmp0_traj
    global dmp1_traj
    global dmp2_traj
    global dmp3_traj
    global dmp4_traj
    
    dmp0_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp1_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp2_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp3_traj = dmp_r_joint_trajectory_client.Trajectory()
    dmp4_traj = dmp_r_joint_trajectory_client.Trajectory()
    with sm:

        smach.StateMachine.add('Go_to_start_position', Go_to_start_position(),
                               transitions={'Succeed':'Go_to_gripper_position'})
        
        smach.StateMachine.add('Go_to_gripper_position',Go_to_gripper_position(),
                               transitions={'Succeed':'Gripper_close'})
                               
        smach.StateMachine.add('Gripper_close',Gripper_close(),
                               transitions={'Succeed':'Go_back'})
        
        smach.StateMachine.add('Go_back',Go_back(),
                               transitions={'Succeed':'Gripper_open'})
        
        smach.StateMachine.add('Gripper_open',Gripper_open(),
                               transitions={'Succeed':'Go_forward'})
        
        smach.StateMachine.add('Go_forward',Go_forward(),
                               transitions={'Succeed':'Go_back_to_start_position'})
        
        smach.StateMachine.add('Go_back_to_start_position',Go_back_to_start_position(),
                               transitions={'Succeed':'Done'})
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())



