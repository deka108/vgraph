#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


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

def draw_line(marker_publisher):
    marker = Marker(
        type=Marker.LINE_LIST,
        action=Marker.ADD,
        ns='vgraph',
        id=1,
        lifetime=rospy.Duration(0),
        # scale=Vector3(0.02, 0, 0),
        color=ColorRGBA(1.0, 0.0, 0.0, 0.8),
        # pose=Pose(Point(0, 0, 0), Quaternion(w=0, x=0, y=0, z=0)),
        header=Header(frame_id='odom'),
    )
    marker.scale.x = 0.02
    marker.points = [Point(0.1, 1, 1e-3), Point(0.01, 1, 1e-3)]
    # marker.points.extend(sss)
    # # RGB (optionally alpha)
    # marker.colors = [std_msg.ColorRGBA(carr[j,0], carr[j,1], carr[j,2], 1.0) 
    #                  ]

    marker_publisher.publish(marker)


def main():
    rospy.init_node('marker_demo')
    wait_for_time()
    marker_publishers = rospy.Publisher('vgraph_markers', Marker, queue_size=5)
    rospy.sleep(0.5)
    # show_text_in_rviz(marker_publisher, 'Hello')
    while not rospy.is_shutdown():
        draw_line(marker_publisher)
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()