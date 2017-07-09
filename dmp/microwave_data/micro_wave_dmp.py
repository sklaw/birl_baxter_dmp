#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun baxter_interface Joint_trajectory_server first!
"""


from birl_sim_examples.srv import *
import argparse
import sys
import rospy
from arm_move.srv_client import *
from arm_move import srv_action_client
import dmp_joint_trajectory_client
import smach
import smach_ros



#class Go_to_button(smach.State):
#    def __init__(self):
#        smach.State.__init__(self,
#                             outcomes=['Succeed'])
#        
#    def execute(self, userdata):
#        rospy.loginfo('executing  Go_to_button...')
#        global dmp0_traj
#        global traj
#        dmp0_traj.parse_file("/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/microwave_data/go_to_button_1_dmp.txt")
#        rospy.loginfo('read file go to button position')
#        dmp0_traj.start()        
#        dmp0_traj.wait()
#        rospy.loginfo('succeesfully run go_to_button_traj')    
#        return 'Succeed'
class Move_door(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing  move door...')
        global dmp0_traj
        global traj
        dmp0_traj.parse_file("/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/microwave_data/move_door_dmp.txt")
        rospy.loginfo('read file go to door position')
        dmp0_traj.start()        
        dmp0_traj.wait()
        rospy.loginfo('succeesfully run move_door_traj')    
        return 'Succeed'

class  Pick_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Pick_object...')
        global limb
        global traj
        global dmp1_traj
        dmp1_traj.parse_file("/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/microwave_data/pick_object_1_dmp.txt")
        dmp1_traj.start()
        dmp1_traj.wait()
        rospy.loginfo('succeesfully run pick_obj traj')
        traj.gripper_close()
        return 'Succeed'

class  Place_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Place_object...')
        global limb
        global dmp2_traj
        dmp2_traj.parse_file("/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/microwave_data/place_object_1_dmp.txt")
        dmp2_traj.start()

        return 'Succeed'


        
def main():
    rospy.init_node("micro_wave_pick_trajectory")
#    rospy.on_shutdown(shutdown)
#    rospy.wait_for_message("/robot/sim/started", Empty)
 
    sm = smach.StateMachine(outcomes=['Done'])
    global traj
    global dmp0_traj
    global dmp1_traj
    global dmp2_traj
    traj = srv_action_client.Trajectory("right")
    dmp0_traj = dmp_joint_trajectory_client.Trajectory()
    dmp1_traj = dmp_joint_trajectory_client.Trajectory()
    dmp2_traj = dmp_joint_trajectory_client.Trajectory()
    with sm:
        smach.StateMachine.add('Move_door',Move_door(),
                               transitions={'Succeed':'Pick_object'})
        
        smach.StateMachine.add('Pick_object',Pick_object(),
                               transitions={'Succeed':'Place_object'})
     
        smach.StateMachine.add('Place_object',Place_object(),
                               transitions={'Succeed': 'Done'})
##        
#        smach.StateMachine.add('Gripper_open',Gripper_open(),
#                               transitions={'Succeed':'Done'})
##        

#                                          'hover_distance':'sm_hover_distance'})
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())


