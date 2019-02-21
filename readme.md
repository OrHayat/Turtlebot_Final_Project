# Turtlebot_Final_Project

assignment running instructions:



0) put the robot paramters in turtlebotbot/navigation/params folder.
1) run the following command in terminal .
	1.1) ssh turtlebot@turtlebot_ip  #when turtlebot_ip is the robot ip
	1.2) export ROS_IP=turtlebot_ip #define ROS_IP and check you defined it right with echo.
	1.3) export ROS_MASTER_URI user_ip #when user_ip is your master pc ip 
		run the following command in the ros.
		"roslaunch  roslaunch turtlebot3-camera turtlebot3_camera.launch 
2)run the following command in terminal .
	source /opt/ros/kentic/setup.bash
    	source $HOME/catkin_ws/devel/setup.bash
   	export ROS_MASTER_URI="http://user_ip:11311"#when user_ip is the host computer ip
	echo $ROS_MASTER_URI# to check that the variable is set correctly
	export ROS_IP=user_ip  #my computer ip. ifconfig
	echo $ROS_IP#to check that the variable is set correcltly
	export TURTLEBOT3_MODEL=burger

	Another Option is to define the following in ~/.bashrc in the robot
	############################################################

		source /opt/ros/kinetic/setup.bash
		source ~/catkin_ws/devel/setup.bash
		export ROS_MASTER_URI=http://user_ip:11311
		export ROS_IP=robot_ip
		export TURTLEBOT3_MODEL=burger
	#after that use source ~/.bashrc in the robot pc
	Another Option is to define the following in ~/.bashrc in the host pc
	############################################################
		source /opt/ros/kinetic/setup.bash
		source ~/catkin_ws/devel/setup.bash
		export ROS_MASTER_URI=http://user_ip:11311
		export ROS_IP=user_ip
		export TURTLEBOT3_MODEL=burger
	############################################################
	#after that use source ~/.bashrc in the hostpc	
3)
	3) map the maze
	3.1)roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
	3.2)roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
	3.3)map the maze with the telop console
	3.4)save the map with the command rosrun map_server map_saver -f ~/map
4) after we defined the env-variables and mapped the maze . run in terminal the following " rosrun project  find_ob.py " and after that run "rosrun project node5.py xpos ypos".
when xpos and ypos is the exit's coordinate in the map.


robot maze solving algotihem:
we use ros navigation stack in order to move the robot to the exit.
meanwhile we have writen node that always check the camera and try to find blue box if it does find it cancel the task of finding the exit and
move the blue box after it move the box the robot send the task to move to the exit again and move to the exit.
