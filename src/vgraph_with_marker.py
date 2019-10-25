#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import rospy
import sys, signal

def signal_handler(signal, frame):
    print("\Exiting program gracefully...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
        type=Marker.TEXT_VIEW_FACING,
        id=0,
        lifetime=rospy.Duration(0),
        pose=Pose(Point(10, 10, 10), Quaternion(w=1, x=0, y=0, z=0)),
        scale=Vector3(10, 10, 10),
        header=Header(frame_id='base_link'),
        color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
        text=text)
    marker_publisher.publish(marker)

def draw_obstacles(marker_publisher, points):
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.header.frame_id = 'odom'
    marker.action = Marker.ADD
    marker.ns = 'obstacles'
    marker.id = 1
    marker.scale.x = 0.02
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    points = [Point(point.x/100.0, point.y/100.0, 1e-3) for point in points ]
    marker.points = points
    marker_publisher.publish(marker)

def draw_djikstra(marker_publisher, points):
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.header.frame_id = 'odom'
    marker.action = Marker.ADD
    marker.ns = 'djikstra'
    marker.id = 1
    marker.scale.x = 0.02
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

    points = [Point(point[0]/100.0, point[1]/100.0, 1e-3) for point in points ]
    marker.points = points
    marker_publisher.publish(marker)

def main():
    rospy.init_node('vgraph_project')

    marker_publisher = rospy.Publisher('vgraph_markers', Marker, queue_size=10)
    rospy.sleep(0.5)
    
    from collections import namedtuple
    Dot = namedtuple('Dot', 'x,y')
    
    # rospy.spin()
    while not rospy.is_shutdown():
        draw_vgraph(marker_publisher, [
            Dot(x=10.0, y=100.0), 
            Dot(x=500.0, y=100.0)
        ])
        # draw_line(marker_publisher)
        # rospy.sleep(0.1)

if __name__ == '__main__':
    main()