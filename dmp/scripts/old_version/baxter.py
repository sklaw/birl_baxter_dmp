#!/usr/bin/env python
import roslib;
roslib.load_manifest('dmp')
import rospy
import numpy as np
import matplotlib.pyplot as plt
from dmp.srv import *
from dmp.msg import *
import baxter_interface

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain,
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
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
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
    rospy.init_node('dmp_tutorial_node')
#    limb_right = baxter_interface.Limb("right")
#    joints_right = limb_right.joint_names()
    plt.close('all')
    # read file
    train_set = np.loadtxt('6_37')


    train_len = len(train_set) #get the lenth 
    resample_t = np.linspace(0.3,train_set[-1,0],train_len)
    joint0_data = np.interp(resample_t, train_set[:,0], train_set[:,9])
    joint1_data = np.interp(resample_t, train_set[:,0], train_set[:,10])
    joint2_data = np.interp(resample_t, train_set[:,0], train_set[:,11])
    joint3_data = np.interp(resample_t, train_set[:,0], train_set[:,12])
    joint4_data = np.interp(resample_t, train_set[:,0], train_set[:,13])
    joint5_data = np.interp(resample_t, train_set[:,0], train_set[:,14])
    joint6_data = np.interp(resample_t, train_set[:,0], train_set[:,15])
    
    traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
    
    for i in range(train_len):
        traj[i] = [joint0_data[i],joint1_data[i],joint2_data[i],joint3_data[i],joint4_data[i],joint5_data[i],joint6_data[i]]

#    joint0_data = train_set[:,9]
#    joint1_data = train_set[:,10]
#    joint2_data = train_set[:,11]
#    joint3_data = train_set[:,12]
#    joint4_data = train_set[:,13]
#    joint5_data = train_set[:,14]
#    joint6_data = train_set[:,15]
#    
    f1, axarr1 = plt.subplots(7, sharex=True)
    axarr1[0].plot(train_set[:,0], joint0_data)
    axarr1[0].set_title('right_arm_joint_space0')
    axarr1[1].plot(train_set[:,0], joint1_data)
    axarr1[2].plot(train_set[:,0], joint2_data)
    axarr1[3].plot(train_set[:,0], joint3_data)
    axarr1[4].plot(train_set[:,0], joint4_data)
    axarr1[5].plot(train_set[:,0], joint5_data)
    axarr1[6].plot(train_set[:,0], joint6_data)
#    traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
#    for i in range(train_len):
#        traj[i] = [joint0_data[i],joint1_data[i],joint2_data[i],joint3_data[i],joint4_data[i],joint5_data[i],joint6_data[i]]
    #plt.show()


    #Create a DMP from a 7-D trajectory
    dims = 7
    dt = 0.01
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 200

    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [joint0_data[0],joint1_data[0],joint2_data[0],
           joint3_data[0], joint4_data[0],joint5_data[0], joint6_data[0]]          #Plan starting at a different point than demo
    x_dot_0 = [0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04]
    t_0 = 1.3
    goal = [ joint0_data[-1], joint1_data[-1], joint2_data[-1],
            joint3_data[-1], joint4_data[-1], joint5_data[-1], joint6_data[-1]]         #Plan to a different goal than demo
    goal_thresh = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
#    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)
    
    
    
    
    
    
    Column0_plan = [0.0]*len(plan.plan.times)
    Column1_plan = [0.0]*len(plan.plan.times)
    Column2_plan = [0.0]*len(plan.plan.times)
    Column3_plan = [0.0]*len(plan.plan.times)
    Column4_plan = [0.0]*len(plan.plan.times)
    Column5_plan = [0.0]*len(plan.plan.times)
    Column6_plan = [0.0]*len(plan.plan.times)
    for i in range(len(plan.plan.times)):    
        Column0_plan[i] = plan.plan.points[i].positions[0]
        Column1_plan[i] = plan.plan.points[i].positions[1]
        Column2_plan[i] = plan.plan.points[i].positions[2]
        Column3_plan[i] = plan.plan.points[i].positions[3]
        Column4_plan[i] = plan.plan.points[i].positions[4]
        Column5_plan[i] = plan.plan.points[i].positions[5]
        Column6_plan[i] = plan.plan.points[i].positions[6]
    
#    plan_len= len(plan.plan.times)
#    resample_t0 = np.linspace(0,train_set[-1,0],plan.plan.times[-1],plan_len)
#    plan_joint0_data = np.interp(resample_t, plan.plan.times, Column0_plan)
#    plan_joint1_data = np.interp(resample_t, plan.plan.times, Column1_plan)
#    plan_joint2_data = np.interp(resample_t, plan.plan.times, Column2_plan)
#    plan_joint3_data = np.interp(resample_t, plan.plan.times, Column3_plan)
#    plan_joint4_data = np.interp(resample_t, plan.plan.times, Column4_plan)
#    plan_joint5_data = np.interp(resample_t, plan.plan.times, Column5_plan)
#    plan_joint6_data = np.interp(resample_t, plan.plan.times, Column6_plan)
##########  record the plan trajectory 
    WriteFileDir ="data01.txt"    
    
    f = open(WriteFileDir,'w')
    f.write('time,')
    f.write('right_s0,')
    f.write('right_s1,')
    f.write('right_e0,')
    f.write('right_e1,')
    f.write('right_w0,')
    f.write('right_w1,')
    f.write('right_w2\n')
        
    for i in range(plan_len):
        f.write("%f," % (plan.plan.times[i],))
        f.write(str(Column0_plan[i])+','+str(Column1_plan[i])+','+str(Column2_plan[i])+','
        +str(Column3_plan[i])+','+str(Column4_plan[i])+','+str(Column5_plan[i])+','+str(Column6_plan[i])
        +'\n')        
    f.close()
###########    
#    
#    print "finished"
    
    f2, axarr2 = plt.subplots(7, sharex=True)
    axarr2[0].plot(resample_t, plan_joint0_data)
    axarr2[0].set_title('right_arm_joint_space1')
    axarr2[1].plot(resample_t, plan_joint1_data)
    axarr2[2].plot(resample_t, plan_joint2_data)
    axarr2[3].plot(resample_t, plan_joint3_data)
    axarr2[4].plot(resample_t, plan_joint4_data)
    axarr2[5].plot(resample_t, plan_joint5_data)
    axarr2[6].plot(resample_t, plan_joint6_data)

#    temp = np.array([0,0,0,0,0,0,0])
#    for traj in range(0, train_len):
#        temp = np.vstack((temp, np.array(plan.plan.points[traj].positions[0:7])))
#    
#    f2, axarr2 = plt.subplots(7, sharex=True)
#    axarr2[0].plot(resample_t, temp[:,0])
#    axarr2[0].set_title('right_arm_joint_space1')
#    axarr2[1].plot(resample_t, temp[:,1])
#    axarr2[2].plot(resample_t, temp[:,2])
#    axarr2[3].plot(resample_t, temp[:,3])
#    axarr2[4].plot(resample_t, temp[:,4])
#    axarr2[5].plot(resample_t, temp[:,5])
#    axarr2[6].plot(resample_t, temp[:,6])
    plt.show()
    