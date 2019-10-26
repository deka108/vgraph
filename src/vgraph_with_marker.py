#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from move_fwd import OutAndBack
from vgraph import main as run_vgraph

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

    points = list(itertools.chain.from_iterable(points)) 
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

    #points = list(itertools.chain.from_iterable(points)) 
    #points = list(itertools.chain.from_iterable(points)) 
    shortest_path = []
    for i in range(len(points) - 1):
      shortest_path.append(points[i])
      shortest_path.append(points[i+1])
    points = [Point(point.x/100.0, point.y/100.0, 1e-3) for point in shortest_path]
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
    points = list(itertools.chain.from_iterable(points)) 
    points = list(itertools.chain.from_iterable(points)) 
    
    points = [Point(point.x/100.0, point.y/100.0, -1e-3) for point in points ]
    marker.points = points
    marker_publisher.publish(marker)

def main():
    rospy.init_node('vgraph_project')

    marker_publisher = rospy.Publisher('vgraph_markers', Marker, queue_size=10)
    print("Growing Obstacles....")
    rospy.sleep(5)
    
    obstacle_segments, free_segments, shortest_path = run_vgraph()

    # rospy.spin()
    while not rospy.is_shutdown():
        # draw the obstacle_segments
        draw_obstacles(marker_publisher, obstacle_segments)
        print("Growth of obstacles done....")
        print("Calculating VGraph....")
        rospy.sleep(5)
        # draw free segments
        draw_vgraph(marker_publisher, free_segments)
        print("Calculation of Vgraph done..")
        print("Calculating shortest path...")
        rospy.sleep(5)
        
        # draw shortest path
        draw_shortest_path(marker_publisher, shortest_path)
        print("Shortest path is", shortest_path)
        rospy.sleep(5)
        print("Moving along shortest path", shortest_path)
        
        shortest_tuples = [ (point.x, point.y) for point in shortest_path]
        t = OutAndBack()
        t.MovePath(shortest_tuples)
        break


if __name__ == '__main__':
    # free_segments = []
    # shortest_path = []

    # with open("free_segments.txt", "r") as fp:
    #     lines = fp.readlines()
    #     for segment in lines:
    #         free_segments.append(ast.literal_eval(segment))

    # with open("shortest_path.txt", 'r') as fp:
    #     lines =  fp.readlines()
    #     for i in range(0, len(lines)-1):
    #         shortest_path.append((ast.literal_eval(lines[i]), ast.literal_eval(lines[i+1])))

    main()
    
    # # print(shortest_path)
    # main(list(itertools.chain.from_iterable(free_segments)), 
    # list(itertools.chain.from_iterable(shortest_path)))
