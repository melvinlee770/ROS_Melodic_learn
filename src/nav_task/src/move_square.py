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
        self.rate = rospy.Rate(10)  # 10 Hz

        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.odom_received = False

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

        # Print real-time position to terminal
        print("Position -> x: {:.2f}, y: {:.2f}, yaw: {:.2f}".format(
            self.pose['x'], self.pose['y'], math.degrees(self.pose['yaw'])))

    def move_straight(self, distance, speed=0.2):
        move_cmd = Twist()
        move_cmd.linear.x = speed

        start_x, start_y = self.pose['x'], self.pose['y']

        while not rospy.is_shutdown():
            current_x, current_y = self.pose['x'], self.pose['y']
            traveled = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

            if traveled >= distance:
                break

            self.pub.publish(move_cmd)
            self.rate.sleep()

        self.pub.publish(Twist())  # Stop

    def rotate(self, angle_deg, angular_speed=0.5):
        turn_cmd = Twist()
        turn_cmd.angular.z = angular_speed

        start_yaw = self.pose['yaw']
        target_yaw = self.normalize_angle(start_yaw + math.radians(angle_deg))

        while not rospy.is_shutdown():
            current_yaw = self.pose['yaw']
            error = self.normalize_angle(target_yaw - current_yaw)

            if abs(error) < 0.02:  # ~1 degree tolerance
                break

            self.pub.publish(turn_cmd)
            self.rate.sleep()

        self.pub.publish(Twist())  # Stop

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_square(self):
        while not self.odom_received:
            rospy.loginfo("Waiting for odometry...")
            self.rate.sleep()

        for i in range(4):
            rospy.loginfo("Side {}: Moving forward".format(i+1))
            self.move_straight(2.0)
            rospy.sleep(1)

            rospy.loginfo("Side {}: Turning".format(i+1))
            self.rotate(90)
            rospy.sleep(1)

        rospy.loginfo("Finished square movement")

if __name__ == '__main__':
    try:
        mover = SquareMover()
        mover.move_square()
    except rospy.ROSInterruptException:
        pass

