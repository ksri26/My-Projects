#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Written by Karan Sridharan
#Performs a robot cleaning operation using the spiral method

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import math
import time

#initially setting the points and angle to 0
x = 0
y = 0
z = 0
yaw = 0

#obtains the spiral motion of the robot
def spiralClean():
	vel_message = Twist()
	#defining parameters to construct the spiral
	count = 0
	constant_speed = 4
	vk = 1
	wk = 2
	rk = 0.5
	loop_rate = rospy.Rate(1)

	while x < 10.5 and y < 10.5: #produces the spiral motion
		rospy.loginfo("The robot is turning")
		rk = rk + 1.0
		vel_message.linear.x =rk
		vel_message.linear.y =0
		vel_message.linear.z =0
		vel_message.angular.x = 0
		vel_message.angular.y = 0
		vel_message.angular.z =constant_speed
		velocity_publisher.publish(vel_message)
		loop_rate.sleep()

	vel_message.linear.x = 0
	velocity_publisher.publish(vel_message)



#gets the callback of the robot's position from the subscriber
def poseCallback(data):
	global x, y, z, yaw
	x= data.x
	y= data.y
	yaw = data.theta


if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim_motion_pose', anonymous=True)
		velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size = 10)
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

		#the below motion commands designs a path for the robot to travel
		spiralClean()

	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
