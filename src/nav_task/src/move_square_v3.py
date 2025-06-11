#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class SquareMover:
    def __init__(self):
        rospy.init_node('turtlebot3_square_mover')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)

        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.odom_received = False
        self.current_heading = 0.0

    def odom_callback(self, msg):
    	# Get position
    	self.pose['x'] = msg.pose.pose.position.x
    	self.pose['y'] = msg.pose.pose.position.y

    	# Get orientation (yaw)
    	orientation_q = msg.pose.pose.orientation
   	_, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y,
                                       orientation_q.z, orientation_q.w])
    	self.pose['yaw'] = yaw
    	self.odom_received = True

    	# Compute yaw error only if heading has been set
    	if hasattr(self, 'current_heading'):
        	yaw_error = self.normalize_angle(self.current_heading - self.pose['yaw'])
        	yaw_error_deg = math.degrees(yaw_error)
    	else:
        	yaw_error_deg = 0.0

    	# Print real-time position + yaw error
    	print("Position -> x: {:.2f}, y: {:.2f}, yaw: {:.2f}, yaw_error: {:.2f}".format(
        	self.pose['x'], self.pose['y'], math.degrees(self.pose['yaw']), yaw_error_deg))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calibrate_yaw(self, target_yaw, angular_speed=0.2):
        # Small correction to align robot to target yaw before moving
        twist = Twist()
        while not rospy.is_shutdown():
            error = self.normalize_angle(target_yaw - self.pose['yaw'])

            if abs(error) < 0.02:  # within ~1 degree
                break

            twist.angular.z = angular_speed if error > 0 else -angular_speed
            self.pub.publish(twist)
            self.rate.sleep()

        self.pub.publish(Twist())

    def move_straight(self, distance, speed=0.2, kp=1.0):
    	move_cmd = Twist()
    	start_x, start_y = self.pose['x'], self.pose['y']

    	while not rospy.is_shutdown():
        	current_x = self.pose['x']
        	current_y = self.pose['y']
        	traveled = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

        	if traveled >= distance:
            		break

        # Yaw correction logic
        	yaw_error = self.normalize_angle(self.current_heading - self.pose['yaw'])
        	angular_correction = kp * yaw_error

        	move_cmd.linear.x = speed
        	move_cmd.angular.z = angular_correction  # Apply correction while driving

        	self.pub.publish(move_cmd)
        	self.rate.sleep()

   	self.pub.publish(Twist())


    def rotate(self, angle_deg, angular_speed=0.5):
        twist = Twist()
        twist.angular.z = angular_speed

        start_yaw = self.pose['yaw']
        target_yaw = self.normalize_angle(start_yaw + math.radians(angle_deg))
        self.current_heading = target_yaw  # Save for calibration

        while not rospy.is_shutdown():
            error = self.normalize_angle(target_yaw - self.pose['yaw'])

            if abs(error) < 0.02:
                break

            twist.angular.z = angular_speed if error > 0 else -angular_speed
            self.pub.publish(twist)
            self.rate.sleep()

        self.pub.publish(Twist())

    def set_yaw_target(self, target_deg):
    	target_rad = self.normalize_angle(math.radians(target_deg))
    	self.current_heading = target_rad


    def move_square(self):
    	while not self.odom_received:
        	rospy.loginfo("Waiting for odometry...")
        	self.rate.sleep()

    	 	# Define absolute yaw targets in degrees
    		yaw_targets = [0, 90, 180, 270]

    		for i in range(4):
        		rospy.loginfo("Calibrating to {} degrees".format(yaw_targets[i]))
        		self.set_yaw_target(yaw_targets[i])
        		self.calibrate_yaw(self.current_heading)

        		rospy.loginfo("Side {}: Moving forward".format(i+1))
        		self.move_straight(2.0)
        		rospy.sleep(1)

    	rospy.loginfo("Finished square movement")


if __name__ == '__main__':
    try:
        mover = SquareMover()
        mover.move_square()
    except rospy.ROSInterruptException:
        pass

