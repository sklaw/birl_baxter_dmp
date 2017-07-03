#!/usr/bin/env python
import rospy
import tf
import sys
#from nav_msgs.msg import Path
#from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    
    rospy.init_node('marker_motion_record')
    listener = tf.TransformListener()

#    pub = rospy.Publisher('marker_motion_topic', Path, queue_size=10)
#    marker_path = Path()
#    marker_PoseStamped = PoseStamped()
    
    # the sampling rate
    rate = rospy.Rate(1.0)

    # file read directory
    ReadFileDir = sys.path[0]+"/../datasets/marker_motion_data9.txt"
    f=open(ReadFileDir,'w')    
    
    # the init time
    init_time = -1
    
    # the user command
    inp = raw_input("Continue? y/n: ")[0]
    if inp != 'y':
        rospy.signal_shutdown("the node shutdown by input cmd")
        
    # lookup the transformation
    listener.waitForTransform("/aruco_marker_582_frame", "/camera_link", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            # set the init time at first time
            if init_time == -1:
                init_time = rospy.Time.now()
                now = init_time
            
            listener.waitForTransform("/aruco_marker_582_frame", "/camera_link", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/aruco_marker_582_frame", "/camera_link", now)
            print "---"
            print "recording the trajectories..."
            
            # write the motion into file
            f.write(str(now-init_time)+' '+
                    str(trans[0])+' '+str(trans[1])+' '+str(trans[2])+' '+
                    str(rot[0])+' '+str(rot[1])+' '+str(rot[2])+' '+str(rot[3])+'\n')
            
#            marker_PoseStamped.header.frame_id = "camera_rgb_optical_frame"
#            marker_PoseStamped.pose.position.x = trans[0]
#            marker_PoseStamped.pose.position.y = trans[1]
#            marker_PoseStamped.pose.position.z = trans[2]
#            marker_PoseStamped.pose.orientation.x = rot[0]
#            marker_PoseStamped.pose.orientation.y = rot[1]
#            marker_PoseStamped.pose.orientation.z = rot[2]
#            marker_PoseStamped.pose.orientation.w = rot[3]
#            marker_path.poses.append(marker_PoseStamped)
#            marker_path.header.frame_id = "camera_rgb_optical_frame"            
#            pub.publish(marker_path)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, KeyboardInterrupt):
            continue
        
        rate.sleep()
