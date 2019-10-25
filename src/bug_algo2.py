#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from tf.transformations import *

from math import *
from sensor_msgs.msg import LaserScan

# Metadata
TWIST_PUBLISHER_NAME = "/cmd_vel_mux/input/teleop"
STEP_MOVEMENT_INTERVAL = 0.2
MINIMUM_OBSTACLE_RANGE = 0.8
GOAL_COORDINATES = (10, 0)
EPS = 10e-3
is_obstacle = False


class OutAndBack:

    def __init__(self):

        # Setup all publishers
        self.cmd_vel = rospy.Publisher(TWIST_PUBLISHER_NAME, Twist, queue_size=5)

        rospy.on_shutdown(self.shutdown)

        self.rate = 50
        self.r = rospy.Rate(self.rate)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        (goal_x, goal_y) = self.normalize_bot()

        self.odom_frame = '/odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
    
   def move_forward(xin,yin,xf,yf):
	
	x1 = xin
	y1 = yin
	x2 = xf
	y2 = yf
	thresh = 0.1
	(position,rotation) = self.get_odom()

	while (abs(x1 - x2) > thresh) and (abs(y1-y2) > thresh):
	    dx = x2-x1
	    dy = y2-y1
	    distTravel = 0.1
	    rotAngle = (degrees(atan2(dy,dx)) - degrees(rotation))
	    self.rotate(rotAngle)
	    self.move(distTravel)
	    (position,rotation) = self.get_odom()
	    x1 = position.x
	    y1 = position.y
	
    def move(self, distance, axis=0):
        print("Moving for: ", distance)
        linear_speed = 1

        move_cmd = Twist()
        if axis == 0:
            move_cmd.linear.x = linear_speed
        if axis == 1:
            move_cmd.linear.y = linear_speed

        if distance < 0:
            move_cmd.linear.x = -1 * linear_speed

        linear_duration = abs(distance / linear_speed)
        ticks = int(linear_duration * self.rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        self.cmd_vel.publish(Twist())

    def rotate(self, goal_angle_degrees):
        print("Request to rotate: ", goal_angle_degrees)
        goal_angle = goal_angle_degrees * pi / 180
        self.rotate_radians(goal_angle)
    
    def rotate_radians(self, goal_angle):

        angular_speed = 1
        angular_tolerance = 0.02

        (position, rotation) = self.get_odom()

        goal_angle += (round(rotation*2/pi) * pi/2) - rotation

        print("Goal Angle: ", goal_angle)
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

        move_cmd.angular.z = angular_speed
        if goal_angle < 0:
            move_cmd.angular.z = -1 * angular_speed

        # Track the last angle measured
        last_angle = rotation

        # Track how far we have turned
        turn_angle = 0

        print("Decision: ", abs(turn_angle + angular_tolerance) < abs(goal_angle))
        while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

            (position, rotation) = self.get_odom()

            delta_angle = normalize_angle(rotation - last_angle)

            turn_angle += delta_angle
            last_angle = rotation

        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

        (position, rotation) = self.get_odom()
        print("Rotation after rotation: ", rotation)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans), quat_to_angle(Quaternion(*rot))

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        t = OutAndBack()
	t.move_forward(0,0,1,1)
	t.move_forward(1,1,2,3)
    except:
        rospy.loginfo("Out-and-Back node terminated.")

