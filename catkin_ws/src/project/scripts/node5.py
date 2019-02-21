#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import sys
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan, CameraInfo
import random
from std_msgs.msg import Bool, Int32, Float32
import numpy as np
import math
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
PI = 3.1415926535897
import image_geometry
import colorsys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
global x,y,goal
def rotate(angle):
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	current_dis=0
	t1=0
	t0=0
	vel_msg= Twist()
	angle2 = abs(angle)
	print("Let's rotate your robot!! angle: "+str(angle))
	speed = angle/7
	angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360
	#We wont use linear components
        vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
       	vel_msg.angular.z = angular_speed
    	t0 = rospy.Time.now().to_sec()
    	current_angle = 0
	r=rospy.Rate(40)
	print "turning"
    	while(  current_angle < abs(relative_angle) and (not rospy.is_shutdown())):
        	velocity_publisher.publish(vel_msg)
        	t1 = rospy.Time.now().to_sec()
		r.sleep()
        	current_angle = abs(angular_speed)*(t1-t0)
        vel_msg.angular.z = abs(0)
        velocity_publisher.publish(vel_msg)
	rospy.sleep(1)

def move_forward():
	data=rospy.wait_for_message("/scan", LaserScan)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	current_dis=0
	t1=0
	t0=0
	print (data.angle_min)
	sum=0
	for i in range(0,11):
		sum=sum+data.ranges[i]
	for i in range(349,360):
		sum=sum+data.ranges[i]
	distance = sum/20
	rospy.loginfo(distance)
	msg= Twist()
	msg.linear.x=0.1
	t0=rospy.Time.now().to_sec()
	r=rospy.Rate(40)
	print "distance avg:"+str(distance)
	distance = distance-0.3
	while((not rospy.is_shutdown()) and current_dis<distance):
		pub.publish(msg)
		t1=rospy.Time.now().to_sec()
		r.sleep()
		current_dis=msg.linear.x*(t1-t0)
			
	print "current dis"+str(current_dis)
	msg.linear.x= 0.0	
	pub.publish(msg)
	return True

def move_forward2():
	data=rospy.wait_for_message("/scan", LaserScan)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	current_dis=0
	t1=0
	t0=0
	print (data.angle_min)
	
	msg= Twist()
	msg.linear.x= 0.1
	t0=rospy.Time.now().to_sec()
	r=rospy.Rate(40)
	moving_distance = 0.5
	print "moving dis"+str(moving_distance)
	while((not rospy.is_shutdown()) and current_dis<moving_distance):
		pub.publish(msg)
		t1=rospy.Time.now().to_sec()
		r.sleep()
		current_dis=msg.linear.x*(t1-t0)
			
	print "current dis"+str(current_dis)
	msg.linear.x= 0.0	
	pub.publish(msg)
	return True
def move_forward3():
	data=rospy.wait_for_message("/scan", LaserScan)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	current_dis=0
	t1=0
	t0=0
	msg= Twist()
	msg.linear.x=0.1
	t0=rospy.Time.now().to_sec()
	r=rospy.Rate(40)
	dist =10
	for i in range(0,16):
		if(data.ranges[i]<dist):
			dist = data.ranges[i]
	for i in range(344,360):
		if(data.ranges[i]<dist):
			dist = data.ranges[i]
	print "distance from obstacle:"+str(dist)
	while((not rospy.is_shutdown()) and current_dis<dist-0.5):
		pub.publish(msg)
		t1=rospy.Time.now().to_sec()
		r.sleep()
		current_dis=msg.linear.x*(t1-t0)
			
	print "current dis"+str(current_dis)
	msg.linear.x= 0.0	
	pub.publish(msg)
	return True
def find_object(r,g,b):


	print "find object"
   	imgdata=rospy.wait_for_message('camera/image_raw',Image)
	if(imgdata is None):
		print "error reciving image from camera"
		return None,None
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(imgdata,"bgr8")
	print "we have image :)"
    	camInfodata=rospy.wait_for_message('camera/camera_info',CameraInfo)
	if(camInfodata is None):
		print "error reciving caminfo"
		return None,None
	camInfo=camInfodata
	print "camInfo:" +str(camInfo.K[0])+  str(camInfo.K[1])+ str(camInfo.K[2])
    	scannerdata=rospy.wait_for_message('scan',LaserScan)
	if(scannerdata is None):
		print "error getting scanner data"
		return None,None
	distances=scannerdata
	if(camInfo!=0 and distances!=0 and len(image)!=0):
		camera = image_geometry.PinholeCameraModel()
		camera.fromCameraInfo(camInfo)
		
		Radius_center=(0,0)
	        Radius_center = findCenter(image,r,g,b)
		if(Radius_center==None):
			print "OBJECT NOT FOUND!"
			msg = Float32()
			msg.data =-1.0
			return  None,None
		else:
			ray = camera.projectPixelTo3dRay(camera.rectifyPoint(Radius_center))
			alpha = np.dot(ray[0],ray[2])
			if(alpha < 0):
				alpha = -alpha
			else:
				alpha = math.floor(math.pi * 2 - alpha)
		 	distance_index = int((alpha - distances.angle_min) / distances.angle_increment)
			actual_distance = distances.ranges[distance_index]
			print "the distance to the object is "+str(actual_distance)
			return actual_distance ,alpha

def findCenter(cv_image,r,g,b):
	  r1 ,g1,b1=colorsys.rgb_to_hsv(r,g,b)
	  red = np.uint8([[[b,g,r ]]])
	  redHSV = cv2.cvtColor(red, cv2.COLOR_BGR2HSV) 
	  greenLower = (redHSV[0][0][0]-25, 80, 80)
	  greenUpper = (redHSV[0][0][0]+25, 255, 255)
	  
	  height, width, channels = cv_image.shape
	  blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
	  hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	  mask = cv2.inRange(hsv, greenLower, greenUpper)
	  mask = cv2.erode(mask, None, iterations=2)
	  mask = cv2.dilate(mask, None, iterations=2)
	  (_,cnts, _)  = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	  center = None
	  if len(cnts) > 0:
	  	c = max(cnts, key=cv2.contourArea)
	  	((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		return (x,y)
	  else:
		return None
def move_to_object():
	data=rospy.wait_for_message("/scan", LaserScan)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	current_dis=0
	t1=0
	t0=0
	center=data.ranges[0]
	print "center move to object "+str (center)
	rospy.loginfo(center)
	msg= Twist()
	if( center>0.7):
		msg.linear.x= 0.1
	else:
		msg.linear.x= 0.0
		pub.publish(msg)
		return False
	t0=rospy.Time.now().to_sec()
	r=rospy.Rate(40)
	while((not rospy.is_shutdown()) and current_dis<0.5):
		pub.publish(msg)
		t1=rospy.Time.now().to_sec()
		r.sleep()
		current_dis=msg.linear.x*(t1-t0)
			
	print "current dis"+str(current_dis)
	msg.linear.x= 0.0	
	pub.publish(msg)
	return True

def move ():
	global r
	global g
	global b
	r=0
	g=0
	b=255
	found=False
	print "move function"
	len, ang=find_object(r,g,b)
	i=0
	print "len before loop ="+str(len)
	while(len is None ):
		moved = move_forward()
		print "moved: "+str(moved)
		rotate(30)
		i+=1
		print "len ="+str(len)
	        len, ang=find_object(r,g,b) 
	print "ang main" +str(ang)
	print "Rotated"
	move_forward2()
	print "moved to object"
def center_obstacle ():
	data=rospy.wait_for_message("/scan", LaserScan)
	r=rospy.Rate(40)
	while(abs(data.ranges[4]-data.ranges[355]>0.2) and data.ranges[4]+data.ranges[355]>0.9):
		if(data.ranges[4] > data.ranges[355]):
			rotate(-3)
		else:
			rotate(3)

		
		
def callback(data):
	global goal
	print "dist.data:" +str(data.data)
	if (data.data !=0.0):
		ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		#wait for the action server to come up
		while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")
		ac.cancel_all_goals()
		
		move_forward3()
		rotate(35)
		move_forward()
		rotate(-125)
		center_obstacle()
		move_forward2()
		ac.send_goal(goal)
		ac.wait_for_result(rospy.Duration(80))
def start(): 
	global x,y,goal
		
	
	goalReached = False
	# initiliaze
	rospy.Rate(40)
	goalReached = moveToGoal(x,y)
	
	print "after sub"
	if(goalReached):
		rospy.loginfo("Congratulations!")
	else:
		rospy.loginfo("Hard Luck!")	
	


def shutdown(self):
# stop turtlebot
	rospy.loginfo("Quit program")
	rospy.sleep()

	
def moveToGoal(xGoal,yGoal):
	global goal
	#define a client for to send goal requests to the move_base server through a SimpleActionClient
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	#wait for the action server to come up
	while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		rospy.loginfo("Waiting for the move_base action server to come up")
	

	goal = MoveBaseGoal()

	#set up the frame parameters
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	# moving towards the goal*/

	goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = 0.0
	goal.target_pose.pose.orientation.w = 1.0

	rospy.loginfo("Sending goal location ...")
	ac.send_goal(goal)
	bool_obstacle = rospy.Subscriber("/obstaclefound",Float32,callback)
	ac.wait_for_result(rospy.Duration(80))

	if(ac.get_state() ==  GoalStatus.SUCCEEDED):
		rospy.loginfo("You have reached the destination")	
		return True

	else:
		rospy.loginfo("The robot failed to reach the destination")
		return False

if __name__ == '__main__':
	try:
		rospy.init_node('map_navigation', anonymous=False)
		global x,y
		x = float(sys.argv[1])
		y= float(sys.argv[2])

		start()
		rospy.spin()
   	except rospy.ROSInterruptException:
		rospy.loginfo("map_navigation node terminated.")
#http://edu.gaitech.hk/turtlebot/map-navigation.html
       
    
