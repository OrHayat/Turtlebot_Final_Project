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




def move_forward():
	data=rospy.wait_for_message("/scan", LaserScan)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	current_dis=0
	t1=0
	t0=0
	print (data.angle_min)
	center=data.ranges[0]
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
	while((not rospy.is_shutdown()) and current_dis<0.3):
		pub.publish(msg)
		t1=rospy.Time.now().to_sec()
		r.sleep()
		current_dis=msg.linear.x*(t1-t0)
			
	print "current dis"+str(current_dis)
	msg.linear.x= 0.0	
	pub.publish(msg)
	return True


def rotate(angle):
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	current_dis=0
	t1=0
	t0=0
	vel_msg= Twist()
	angle = abs(angle)
	if(angle>10):
		angle=angle+5
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
       	vel_msg.angular.z = abs(angular_speed)
    	t0 = rospy.Time.now().to_sec()
    	current_angle = 0
	r=rospy.Rate(40)
	print "turning"
    	while(  current_angle < relative_angle and (not rospy.is_shutdown())):
        	velocity_publisher.publish(vel_msg)
        	t1 = rospy.Time.now().to_sec()
		r.sleep()
        	current_angle = angular_speed*(t1-t0)
        vel_msg.angular.z = abs(0)
        velocity_publisher.publish(vel_msg)
	rospy.sleep(1)
 
def find_object():
	rospy.Rate(40)
	publisher = rospy.Publisher('/obstaclefound', Float32, queue_size=10)
	dist = Float32()
	dist.data = 0.0
	while(dist.data ==0.0):
	   	imgdata=rospy.wait_for_message('usb_cam/image_raw',Image)
		if(imgdata is None):
			return None,None
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(imgdata,"bgr8")
	    	camInfodata=rospy.wait_for_message('usb_cam/camera_info',CameraInfo)
		if(camInfodata is None):
			return None,None
		camInfo=camInfodata
	    	scannerdata=rospy.wait_for_message('scan',LaserScan)
		if(scannerdata is None):
			return None,None
		distances=scannerdata
		if(distances!=0 and len(cv_image)!=0):
			camera = image_geometry.PinholeCameraModel()
			camera.fromCameraInfo(camInfo)
		
			Radius_center=(0,0)
			Radius_center = findCenter(cv_image)
			if(Radius_center==None):
				print "OBJECT NOT FOUND!"
				msg = Float32()
				msg.data =-1.0
			else:
				ray = camera.projectPixelTo3dRay(camera.rectifyPoint(Radius_center))
				alpha = np.dot(ray[0],ray[2])
				if(alpha < 0):
					alpha = -alpha
				else:
					alpha = math.floor(math.pi * 2 - alpha)
			 	distance_index = int((alpha - distances.angle_min)/distances.angle_increment)
				actual_distance = distances.ranges[distance_index]
				print "the distance to the object is "+str(actual_distance)
				dist.data = actual_distance
				publisher.publish(dist)
				



def findCenter(cv_image):
	red = np.uint8([[[240, 0,0 ]]])
	redHSV = cv2.cvtColor(red, cv2.COLOR_BGR2HSV) 
	greenLower = (redHSV[0][0][0]-25, 50, 50)
	greenUpper = (redHSV[0][0][0]+25, 255, 255)

	height, width, channels = cv_image.shape
	blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	(_,cnts1, _)  = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cXreal = None
	cYreal = None
	perimeterMax = 0
	print len(cnts1)
	for c in cnts1:
		# compute the center of the contour
		M = cv2.moments(c)
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
			perimeter = cv2.arcLength(c,True)
			if(perimeter > perimeterMax):
				cXreal = cX
				cYreal = cY
				perimeterMax = perimeter

	radius_center= (cXreal,cYreal)

	if(perimeterMax <950):
		return None
	print "perimeter is: "+str(perimeterMax)
	print "radius is: "+str(cX)+" , "+str(cY)
	for c in cnts1:
		cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
		cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
	return radius_center

def handle_obstacle():
	find_object()
	

if __name__ == '__main__':
	try:
		rospy.init_node('obstacle', anonymous=True)
		handle_obstacle()
		rospy.spin()
   	except rospy.ROSInterruptException:
		rospy.loginfo("map_navigation node terminated.")
