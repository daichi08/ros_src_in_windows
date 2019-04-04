#! /usr/bin/env python
# coding: utf-8

import rospy
import math
import numpy as np
import random
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Point():
    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

obstacles = []
offset    = 90 * math.pi/180.0

def callback(msg):
    global obstacles
    obstacles     = []
    linear_points = []
    datasize  = len(msg.ranges)
    angle_inc = msg.angle_increment

    before_point = Point(msg.ranges[0]*math.cos(msg.angle_min),msg.ranges[0]*math.sin(msg.angle_min))

    for i in range(datasize):
        angle = msg.angle_min + angle_inc * i
        current_vector = msg.ranges[i]
        if not math.isnan(current_vector):
            current_point = Point(current_vector * math.cos(angle), current_vector * math.sin(angle))
            norm = math.sqrt(pow(current_point.x-before_point.x, 2)+pow(current_point.y-before_point.y, 2))
            similarity = 1/(1+norm)

            if similarity > 0.95:
                linear_points.append(current_point)
            else:
                obstacles.append(linear_points)
                linear_points = []
        before_point = current_point

def main():
    rospy.init_node("scan_values")
    r = rospy.Rate(20)
    sub = rospy.Subscriber("/scan", LaserScan, callback, queue_size=1)
    pub = rospy.Publisher("/markers_pi", MarkerArray, queue_size=2)
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        index = 0

        if obstacles:
            for obstacle in obstacles:
                if obstacle:
                    marker = Marker()

                    marker.header.frame_id = "/laser"
                    marker.id = index
                    marker.ns = "object"
                    marker.type = marker.LINE_LIST
                    marker.action = marker.ADD

                    #marker.pose.position.x = 0
                    #marker.pose.position.y = 0
                    #marker.pose.position.z = 0

                    marker.pose.orientation.x = 0
                    marker.pose.orientation.y = 0
                    marker.pose.orientation.z = 0
                    marker.pose.orientation.w = 1

                    marker.points.append(obstacle[0])
                    marker.points.append(obstacle[-1])
                    #marker.points.append(Point(0,0))
                    #marker.points.append(Point(random.random(),random.random()))

                    marker.color.r = 1
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5

                    marker.scale.x = 0.01
                    marker.scale.y = 0
                    marker.scale.z = 0

                    marker.lifetime = rospy.Duration(1)

                    marker_array.markers.append(marker)

                    index += 1
            pub.publish(marker_array)
        else:
            print("wait")
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
