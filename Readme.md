#baxter_dmp wiki
Open the head camera on baxter

	rosrun baxter_tools camera_control.py -l
	rosrun baxter_tools camera_control.py -c left_hand_camera
	rosrun baxter_tools camera_control.py -o head_camera

Using camera recognize the marker to get the position of the goal

	roslaunch aruco_ros baxter_camera.launch 
	
Generate dmp trajectory

	 roslaunch dmp baxter_r_arm_dmp.launch

Run the code in baxter
	
	roslaunch dmp baxter_r_arm_move.launch
