#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import ast
import itertools
import rospy
import sys, signal

def signal_handler(signal, frame):
    print("\Exiting program gracefully...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def draw_obstacles(marker_publisher, points):
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.header.frame_id = 'odom'
    marker.action = Marker.ADD
    marker.ns = 'obstacles'
    marker.id = 1
    marker.scale.x = 0.02
    marker.color.a = 0.8
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    points = [Point(point.x/100.0, point.y/100.0, -1e-3) for point in points ]
    marker.points = points
    marker_publisher.publish(marker)

def draw_shortest_path(marker_publisher, points):
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.header.frame_id = 'odom'
    marker.action = Marker.ADD
    marker.ns = 'shortest_path'
    marker.id = 1
    marker.scale.x = 0.03
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    points = [Point(point[0]/100.0, point[1]/100.0, 1e-3) for point in points]
    marker.points = points
    marker_publisher.publish(marker)

def draw_vgraph(marker_publisher, points):
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.header.frame_id = 'odom'
    marker.action = Marker.ADD
    marker.ns = 'vgraph'
    marker.id = 1
    marker.scale.x = 0.02
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    points = [Point(point[0]/100.0, point[1]/100.0, -1e-3) for point in points ]
    marker.points = points
    marker_publisher.publish(marker)

def main(free_segments, shortest_path):
    rospy.init_node('vgraph_project')

    marker_publisher = rospy.Publisher('vgraph_markers', Marker, queue_size=10)
    rospy.sleep(0.5)
    
    # rospy.spin()
    while not rospy.is_shutdown():
        # draw free segments
        draw_vgraph(marker_publisher, free_segments)
        
        # draw shortest path
        draw_shortest_path(marker_publisher, shortest_path)
        rospy.sleep(0.1)

if __name__ == '__main__':
    free_segments = []
    shortest_path = []

    with open("free_segments.txt", "r") as fp:
        lines = fp.readlines()
        for segment in lines:
            free_segments.append(ast.literal_eval(segment))

    with open("shortest_path.txt", 'r') as fp:
        lines =  fp.readlines()
        for i in range(0, len(lines)-1):
            shortest_path.append((ast.literal_eval(lines[i]), ast.literal_eval(lines[i+1])))
    
    # print(shortest_path)
    main(list(itertools.chain.from_iterable(free_segments)), 
    list(itertools.chain.from_iterable(shortest_path)))
