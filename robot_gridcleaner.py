#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Written by Karan Sridharan
#Performs a robot cleaning operation using the grid method

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


#gets the callback of the robot's position from the subscriber
def poseCallback(data):
	global x, y, z, yaw
	x= data.x
	y= data.y
	yaw = data.theta


#gets the forward and backward motion of the robot
def move(speed, distance, isForward):
	vel_message = Twist()
	x0 = x
	y0 = y
	rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	loop_rate = rospy.Rate(100) #determines whether the robot should move front or back
	if isForward == 0:
		vel_message.linear.x = abs(speed)
	else:
		vel_message.linear.x = -abs(speed)
	vel_message.linear.y = 0
	vel_message.linear.z = 0
	vel_message.angular.x = 0
	vel_message.angular.y = 0
	vel_message.angular.z =0

	distance_moved = 0.0
	while distance_moved < distance: #produces the motion of the robot
		rospy.loginfo("The robot moves forward")
		velocity_publisher.publish(vel_message)
		loop_rate.sleep()
		distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
		print('Distance moved')
		if distance_moved > distance:
			rospy.loginfo('Robot has reached the destination')
			break

#gets the left/right turn motion of the robot
def rotate(angular_speed, relative_angle, clockwise):
	vel_message = Twist()
	rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	loop_rate = rospy.Rate(100)
	if clockwise == 0: #determines whether the robot should move left or right
		vel_message.angular.z = -abs(angular_speed)
	else:
		vel_message.angular.z = abs(angular_speed)
	vel_message.linear.y = 0
	vel_message.linear.z = 0
	vel_message.angular.x = 0
	vel_message.angular.y = 0

	current_angle = 0.0

	t0 = 0.0

	while current_angle < relative_angle: #produces the turning motion of the robot
		rospy.loginfo("The robot is turning")
		velocity_publisher.publish(vel_message)
		loop_rate.sleep()
		current_angle = current_angle + angular_speed*0.01111
		if current_angle > relative_angle:
			rospy.loginfo("The robot has stopped turning")
			break

#converts the angles to radians
def degrees2radians(angle_in_degrees):
	return angle_in_degrees*math.pi/180

#helps to set to whichever orientation the robot should be 
def setDesiredOrientation(desired_angle_radians):
	relative_angle_radians = desired_angle_radians - yaw
	if relative_angle_radians < 0:
		clockwise = 1
	else:
		clockwise = 0
	rotate(degrees2radians(10), abs(relative_angle_radians), 1.0)

#gets the destination position the robot shoud be at
def moveGoal(goal_x, goal_y, distance_tolerance):
	global x
	global y, z, yaw
	vel_message = Twist()
	rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	loop_rate = rospy.Rate(100)
	x0 = goal_x
	y0 = goal_y
	while True: #produces the motion of the robot to move to the goal position
		rospy.loginfo("The robot is reaching its starting goal")
		Kp = 1.0
		Ki = 0.02
		distance_moved = abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2))) 
		vel_message.linear.x = (Kp*distance_moved);
		vel_message.linear.y =0;
		vel_message.linear.z =0;
		vel_message.angular.x = 0;
		vel_message.angular.y = 0;
		vel_message.angular.z = 4*(math.atan2(y0-y,x0-x)-yaw) #formula to identify the angular velocity of the z axis
		velocity_publisher.publish(vel_message)
		loop_rate.sleep()
		if distance_moved < distance_tolerance:
			rospy.loginfo("The robot has reached its starting goal")
			break


if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim_motion_pose', anonymous=True)
		velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size = 10)
		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
		#the below motion commands designs a path for the robot to travel
		moveGoal(1.0, 1.0, 0.01)
		time.sleep(1)
		setDesiredOrientation(0)
		time.sleep(1)

		move (2.0, 9.0, 0.0)
		time.sleep(1)
		rotate (degrees2radians(10),degrees2radians(90),1.0)
		time.sleep(1)
		move (2.0, 9.0, 0.0)
		time.sleep(1)

		rotate (degrees2radians(10),degrees2radians(90),1.0)
		time.sleep(1)
		move (2.0, 9.0, 0.0)
		time.sleep(1)
		rotate (degrees2radians(10),degrees2radians(90),1.0)
		time.sleep(1)
		move (2.0, 9.0, 0.0)
		time.sleep(1)

		rotate (degrees2radians(10),degrees2radians(90),1.0)
		time.sleep(1)
		move (2.0, 9.0, 0.0)
		time.sleep(1)

		rospy.wait_for_service('reset')
		reset_turtle = rospy.ServiceProxy('reset', Empty)
		reset_turtle()
		print('end reset:')
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
