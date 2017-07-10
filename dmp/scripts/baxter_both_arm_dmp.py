#!/usr/bin/env python
import roslib;
import sys
sys.path[0]="/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp"
roslib.load_manifest('dmp')
import rospy
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 
from dmp.srv import *
from dmp.msg import *
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState




class Dmp(object):
    def __init__(self):
         self.q_s0 = None
         self.q_s1 = None
         self.q_e0 = None
         self.q_e1 = None
         self.q_w0 = None
         self.q_w1 = None
         self.q_w2 = None
  
    def callback(self,data):    
        self.q_s0 = data.command[0]
        self.q_s1 = data.command[1]
        self.q_e0 = data.command[2]
        self.q_e1= data.command[3]
        self.q_w0= data.command[4]
        self.q_w1= data.command[5]
        self.q_w2= data.command[6]
        rospy.loginfo("q_s0 %s", data.command[0])
        rospy.loginfo("q_s1 %s", data.command[1])
        rospy.loginfo("q_e0 %s", data.command[2])
        rospy.loginfo("q_e1 %s", data.command[3])
        rospy.loginfo("q_w0 %s", data.command[4])
        rospy.loginfo("q_w1 %s", data.command[5])
        rospy.loginfo("q_w2 %s", data.command[6])
    
    def setpoint_callback(self,data):
        self.j_s0 = data.position[9]
        self.j_s0 = data.position[10]
        self.j_s0 = data.position[11]
        self.j_s0 = data.position[12]
        self.j_s0 = data.position[13]
        self.j_s0 = data.position[14]
        self.j_s0 = data.position[15]
        self.j_s0 = data.position[16]
        
          
#Learn a DMP from demonstration data
    def makeLFDRequest(self,dims, traj, dt, K_gain,
                       D_gain, num_bases):
        demotraj = DMPTraj()
    
        for i in range(len(traj)):
            pt = DMPPoint();
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
    
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
    
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"
    
        return resp;


#Set a DMP as active for planning
    def makeSetActiveRequest(self,dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


#Generate a plan from a DMP
    def makePlanRequest(self,x_0, x_dot_0, t_0, goal, goal_thresh,
                        seg_length, tau, dt, integrate_iter):
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                       seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"
    
        return resp;


if __name__ == '__main__':
    rospy.init_node('dmp_baxter_r_arm_node')
    dmp = Dmp()
#    rospy.Subscriber("end_effector_command_solution",JointCommand, dmp.callback)   # track_ik
#    rospy.wait_for_message("end_effector_command_solution",JointCommand)
    
#    rospy.loginfo("q_s0 %s", dmp.q_s0)
    #rospy.Subscriber("robot/joint_states", sensor_msgs/JointState, dmp.setpoint_callback)

    plt.close('all')
    
    
    # read file
    train_set = pd.read_csv('/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/datasets/go_to_place_position.txt')
    


    train_len = len(train_set)
    resample_t = np.linspace(train_set.values[0,0],train_set.values[-1,0],train_len)
    r_joint0_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,9])
    r_joint1_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,10])
    r_joint2_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,11])
    r_joint3_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,12])
    r_joint4_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,13])
    r_joint5_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,14])
    r_joint6_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,15])
    
    l_joint0_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,1])
    l_joint1_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,2])
    l_joint2_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,3])
    l_joint3_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,4])
    l_joint4_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,5])
    l_joint5_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,6])
    l_joint6_data = np.interp(resample_t, train_set.values[:,0], train_set.values[:,7])
    
    r_traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
    l_traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
    
    for i in range(train_len):
        r_traj[i] = [r_joint0_data[i],r_joint1_data[i],r_joint2_data[i],r_joint3_data[i],r_joint4_data[i],r_joint5_data[i],r_joint6_data[i]]
    
    for i in range(train_len):
        l_traj[i] = [l_joint0_data[i],l_joint1_data[i],l_joint2_data[i],l_joint3_data[i],l_joint4_data[i],l_joint5_data[i],l_joint6_data[i]]

    
    f1, axarr1 = plt.subplots(7, sharex=True)
    axarr1[0].plot(resample_t, r_joint0_data)
    axarr1[0].set_title('right_arm_joint_space0')
    axarr1[1].plot(resample_t, r_joint1_data)
    axarr1[2].plot(resample_t, r_joint2_data)
    axarr1[3].plot(resample_t, r_joint3_data)
    axarr1[4].plot(resample_t, r_joint4_data)
    axarr1[5].plot(resample_t, r_joint5_data)
    axarr1[6].plot(resample_t, r_joint6_data)
    
    f2, axarr2 = plt.subplots(7, sharex=True)
    axarr2[0].plot(resample_t, l_joint0_data)
    axarr2[0].set_title('left_arm_joint_space0')
    axarr2[1].plot(resample_t, l_joint1_data)
    axarr2[2].plot(resample_t, l_joint2_data)
    axarr2[3].plot(resample_t, l_joint3_data)
    axarr2[4].plot(resample_t, l_joint4_data)
    axarr2[5].plot(resample_t, l_joint5_data)
    axarr2[6].plot(resample_t, l_joint6_data)

    #plt.show()


    #Create a DMP from a 7-D trajectory
    dims = 7
    dt = 0.01
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 200

    resp_right = dmp.makeLFDRequest(dims, r_traj, dt, K, D, num_bases)
    resp_left = dmp.makeLFDRequest(dims, l_traj, dt, K, D, num_bases)

    #Set it as the active DMP
    dmp.makeSetActiveRequest(resp_right.dmp_list)
    dmp.makeSetActiveRequest(resp_right.dmp_list)

    #Now, generate a plan
    x_0_r_arm = [r_joint0_data[0],r_joint1_data[0],r_joint2_data[0],
           r_joint3_data[0], r_joint4_data[0],r_joint5_data[0], r_joint6_data[0]]          #Plan starting at a different point than demo

    x_0_l_arm = [l_joint0_data[0],l_joint1_data[0],l_joint2_data[0],
           l_joint3_data[0], l_joint4_data[0],l_joint5_data[0], l_joint6_data[0]] 
    
    x_dot_0 = [0.4, 0.4, 0.4, 0.4, 0.4, 0.0, 0.4]
    
    t_0 = train_set.values[0,0] # better to choose the starting time in the record file
#    
#    goal = [ joint_angles['right_s0'], joint_angles['right_s1'],         
#             joint_angles['right_e0'], joint_angles['right_e1'],
#             joint_angles['right_w0'], joint_angles['right_w1'],
#             joint_angles['right_w2']]         #Plan to a different goal than demo
#    goal = [dmp.q_s0, 
#            dmp.q_s1, 
#            dmp.q_e0,
#           dmp.q_e1, 
#            dmp.q_w0, 
#           dmp.q_w1, 
#            dmp.q_w2 ]
    r_arm_goal =[ r_joint0_data[-1],r_joint1_data[-1],
    r_joint2_data[-1], r_joint3_data[-1],r_joint4_data[-1],r_joint5_data[-1], r_joint6_data[-1]         ]
    
    l_arm_goal =[ l_joint0_data[-1],l_joint1_data[-1],
    l_joint2_data[-1], l_joint3_data[-1],l_joint4_data[-1],l_joint5_data[-1], l_joint6_data[-1]         ]
    
    goal_thresh = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]   
    
    seg_length = -1          #Plan until convergence to goal
    
    r_tau = 1 * resp_right.tau
    l_tau = 1 * resp_left.tau
      #Desired plan should take twice as long as demo #let see change to 1
#    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1
    
    r_arm_plan = dmp.makePlanRequest(x_0_r_arm, x_dot_0, t_0, r_arm_goal, goal_thresh,
                           seg_length, r_tau, dt, integrate_iter)
    
    l_arm_plan = dmp.makePlanRequest(x_0_l_arm, x_dot_0, t_0, l_arm_goal, goal_thresh,
                           seg_length, l_tau, dt, integrate_iter)
    
    
    
####################################### finish dmp #################################################################    
    
####################################### plot the curve generated by dmp #################################################################
    
    r_Column0_plan = [0.0]*len(r_arm_plan.plan.times)
    r_Column1_plan = [0.0]*len(r_arm_plan.plan.times)
    r_Column2_plan = [0.0]*len(r_arm_plan.plan.times)
    r_Column3_plan = [0.0]*len(r_arm_plan.plan.times)
    r_Column4_plan = [0.0]*len(r_arm_plan.plan.times)
    r_Column5_plan = [0.0]*len(r_arm_plan.plan.times)
    r_Column6_plan = [0.0]*len(r_arm_plan.plan.times)
    for i in range(len(r_arm_plan.plan.times)):    
        r_Column0_plan[i] = r_arm_plan.plan.points[i].positions[0]
        r_Column1_plan[i] = r_arm_plan.plan.points[i].positions[1]
        r_Column2_plan[i] = r_arm_plan.plan.points[i].positions[2]
        r_Column3_plan[i] = r_arm_plan.plan.points[i].positions[3]
        r_Column4_plan[i] = r_arm_plan.plan.points[i].positions[4]
        r_Column5_plan[i] = r_arm_plan.plan.points[i].positions[5]
        r_Column6_plan[i] = r_arm_plan.plan.points[i].positions[6]
        
    resample_t0 = np.linspace(0.01,r_arm_plan.plan.times[-1], train_len)
    r_joint0_data_plan = np.interp(resample_t0, r_arm_plan.plan.times, r_Column0_plan)
    r_joint1_data_plan = np.interp(resample_t0, r_arm_plan.plan.times, r_Column1_plan)
    r_joint2_data_plan = np.interp(resample_t0, r_arm_plan.plan.times, r_Column2_plan)
    r_joint3_data_plan = np.interp(resample_t0, r_arm_plan.plan.times, r_Column3_plan)
    r_joint4_data_plan = np.interp(resample_t0, r_arm_plan.plan.times, r_Column4_plan)
    r_joint5_data_plan = np.interp(resample_t0, r_arm_plan.plan.times, r_Column5_plan)
    r_joint6_data_plan = np.interp(resample_t0, r_arm_plan.plan.times, r_Column6_plan)
    
#############################################################################    
   
    l_Column0_plan = [0.0]*len(l_arm_plan.plan.times)
    l_Column1_plan = [0.0]*len(l_arm_plan.plan.times)
    l_Column2_plan = [0.0]*len(l_arm_plan.plan.times)
    l_Column3_plan = [0.0]*len(l_arm_plan.plan.times)
    l_Column4_plan = [0.0]*len(l_arm_plan.plan.times)
    l_Column5_plan = [0.0]*len(l_arm_plan.plan.times)
    l_Column6_plan = [0.0]*len(l_arm_plan.plan.times)
    for i in range(len(l_arm_plan.plan.times)):    
        l_Column0_plan[i] = l_arm_plan.plan.points[i].positions[0]
        l_Column1_plan[i] = l_arm_plan.plan.points[i].positions[1]
        l_Column2_plan[i] = l_arm_plan.plan.points[i].positions[2]
        l_Column3_plan[i] = l_arm_plan.plan.points[i].positions[3]
        l_Column4_plan[i] = l_arm_plan.plan.points[i].positions[4]
        l_Column5_plan[i] = l_arm_plan.plan.points[i].positions[5]
        l_Column6_plan[i] = l_arm_plan.plan.points[i].positions[6]
   
    resample_t0 = np.linspace(0.01,l_arm_plan.plan.times[-1], train_len)    
    l_joint0_data_plan = np.interp(resample_t0, l_arm_plan.plan.times, l_Column0_plan)
    l_joint1_data_plan = np.interp(resample_t0, l_arm_plan.plan.times, l_Column1_plan)
    l_joint2_data_plan = np.interp(resample_t0, l_arm_plan.plan.times, l_Column2_plan)
    l_joint3_data_plan = np.interp(resample_t0, l_arm_plan.plan.times, l_Column3_plan)
    l_joint4_data_plan = np.interp(resample_t0, l_arm_plan.plan.times, l_Column4_plan)
    l_joint5_data_plan = np.interp(resample_t0, l_arm_plan.plan.times, l_Column5_plan)
    l_joint6_data_plan = np.interp(resample_t0, l_arm_plan.plan.times, l_Column6_plan)
##########  record the plan trajectory 
    WriteFileDir ="/home/tony/ros/indigo/baxter_ws/src/birl_baxter/birl_baxter_dmp/dmp/datasets/go_to_place_position_dmp.txt"    ## the path of generated dmp traj
    plan_len = len(r_arm_plan.plan.times)
    f = open(WriteFileDir,'w')
    f.write('time,')
    f.write('left_s0,')
    f.write('left_s1,')
    f.write('left_e0,')
    f.write('left_e1,')
    f.write('left_w0,')
    f.write('left_w1,')
    f.write('left_w2,')
    f.write('right_s0,')
    f.write('right_s1,')
    f.write('right_e0,')
    f.write('right_e1,')
    f.write('right_w0,')
    f.write('right_w1,')
    f.write('right_w2\n')
        
    for i in range(train_len):
        f.write("%f," % (resample_t[i],))
        f.write(str(l_joint0_data_plan[i])+','+str(l_joint1_data_plan[i])+','+str(l_joint2_data_plan[i])+','
        +str(l_joint3_data_plan[i])+','+str(l_joint4_data_plan[i])+','+str(l_joint5_data_plan[i])+','+str(l_joint6_data_plan[i])
        +','+str(r_joint0_data_plan[i])+','+str(r_joint1_data_plan[i])+','+str(r_joint2_data_plan[i])+','+str(r_joint3_data_plan[i])
        +','+str(r_joint4_data_plan[i])+','+str(r_joint5_data_plan[i])+','+str(r_joint6_data_plan[i])
        +'\n')        
    f.close()
###########    
#    
#    print "finished"

########### plot    
    f3, axarr3 = plt.subplots(7, sharex=True)
    axarr3[0].plot(resample_t, r_joint0_data_plan)
    axarr3[0].set_title('right_arm_joint_space1')
    axarr3[1].plot(resample_t, r_joint1_data_plan)
    axarr3[2].plot(resample_t, r_joint2_data_plan)
    axarr3[3].plot(resample_t, r_joint3_data_plan)
    axarr3[4].plot(resample_t, r_joint4_data_plan)
    axarr3[5].plot(resample_t, r_joint5_data_plan)
    axarr3[6].plot(resample_t, r_joint6_data_plan)
    
    f4, axarr4 = plt.subplots(7, sharex=True)
    axarr4[0].plot(resample_t, l_joint0_data_plan)
    axarr4[0].set_title('left_arm_joint_space1')
    axarr4[1].plot(resample_t, l_joint1_data_plan)
    axarr4[2].plot(resample_t, l_joint2_data_plan)
    axarr4[3].plot(resample_t, l_joint3_data_plan)
    axarr4[4].plot(resample_t, l_joint4_data_plan)
    axarr4[5].plot(resample_t, l_joint5_data_plan)
    axarr4[6].plot(resample_t, l_joint6_data_plan)



    plt.show()

