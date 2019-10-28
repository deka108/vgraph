#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from tf.transformations import *

from math import *
from sensor_msgs.msg import LaserScan

# Metadata
TWIST_PUBLISHER_NAME = '/cmd_vel'

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
    
    def move_forward(self, xin,yin,xf,yf):
        #Compute angle to rotate followed by linear distance to move
        x1 = xin
        y1 = yin
        x2 = xf
        y2 = yf
        dx = x2-x1
        dy = y2-y1
        distTravel = sqrt((dx*dx) + (dy*dy))
        (position,rotation) = self.get_odom()
        rotAngle = (degrees(atan2(dy,dx)) - degrees(rotation))
        print("Will rotate by an angle of",rotAngle)
        self.rotate(rotAngle)
        self.move(distTravel)
        self.cmd_vel.publish(Twist())
	
    def move(self, distance, axis=0):
        #Function responsible for linear translation.. This will move straight
        #along the direction in which the bot is oriented

        print("Moving for: ", distance)
        linear_speed = 0.5

        move_cmd = Twist()
        if axis == 0:
            move_cmd.linear.x = linear_speed
        if axis == 1:
            move_cmd.linear.y = linear_speed

        if distance < 0:
            move_cmd.linear.x = -1 * linear_speed

        goal_distance = distance

        # Get the starting position values     
        (position, rotation) = self.get_odom()
        
        x_start = position.x
        y_start = position.y
        
        # Keep track of the distance traveled
        distance = 0
        
        # Enter the loop to move along a side
        while distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            self.r.sleep()
        
            # Get the current position
            (position, rotation) = self.get_odom()
            
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))

        self.cmd_vel.publish(Twist())

    def rotate(self, goal_angle_degrees):
        #Wrapper function which takes as input degrees to rotate and passes 
        #converted radians to the rotate_radians function
        goal_angle = goal_angle_degrees * pi / 180
        self.rotate_radians(goal_angle)
    
    def rotate_radians(self, goal_angle):

        angular_speed = 0.5
        angular_tolerance = 0.02

        (position, rotation) = self.get_odom()

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

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans), quat_to_angle(Quaternion(*rot))

    def MovePath(self,coords):
        #Function that takes as input a list of coordinates which represent the
        #path that needs to be traversed and traverses the path one edge at a time
        numPts = len(coords)
        for x in range((numPts-1)):
            x1,y1 = coords[x]
            x2,y2 = coords[(x+1)]
            self.move_forward(x1/100.0,y1/100.0,x2/100.0,y2/100.0)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        t = OutAndBack()
        coords = [(0,0),(82, 68),(118, 68),(293, 43),(432, 12),(600, 0)]
        t.MovePath(coords)
    except :
        rospy.loginfo("Out-and-Back node terminated.")
        raise
