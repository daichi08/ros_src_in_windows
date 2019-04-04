#! /usr/bin/env python
# coding: utf-8

import rospy
import math
import numpy as np
import random
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

obstacles = []

def callback(msg):
    global obstacles

    index     = 0
    SIM_LIMIT = 0.95
    datasize  = len(msg.ranges)
    rad_inc   = msg.angle_increment
    rad_min   = msg.angle_min

    obstacles     = []
    linear_points = []
    before_point = [
        msg.ranges[0]*math.cos(rad_min),
        msg.ranges[0]*math.sin(rad_min)
    ]

    for range_ in msg.ranges:
        angle = rad_min + rad_inc * index
        if not math.isnan(range_):
            current_point = [
                range_ * math.cos(angle),
                range_ * math.sin(angle)
            ]
            norm = math.sqrt(
                pow(current_point[0] - before_point[0], 2)+
                pow(current_point[1] - before_point[1], 2)
            )
            similarity = 1/(1+norm)

            if index == datasize-1:
                if similarity > SIM_LIMIT:
                    linear_points.append(current_point)
                obstacles.append(linear_points)
                linear_points = []
            elif similarity > SIM_LIMIT:
                linear_points.append(current_point)
            else:
                obstacles.append(linear_points)
                linear_points = []
        before_point = current_point
        index += 1

def main():
    rospy.init_node("scan_values")
    r   = rospy.Rate(20)
    sub = rospy.Subscriber("/scan", LaserScan, callback, queue_size=1)
    pub = rospy.Publisher("/markers_py", MarkerArray, queue_size=2)
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        index = 0

        if obstacles:
            for obstacle in obstacles:
                if obstacle:
                    marker      = Marker()
                    first_point = Point()
                    last_point  = Point()

                    tmp_first_point = obstacle[0]
                    tmp_last_point  = obstacle[-1]

                    first_point.x = tmp_first_point[0]
                    first_point.y = tmp_first_point[1]
                    first_point.z = 0
                    last_point.x  = tmp_last_point[0]
                    last_point.y  = tmp_last_point[1]
                    last_point.z  = 0

                    marker.header.frame_id = "/laser"
                    marker.id = index
                    marker.ns = "object"
                    marker.type = marker.LINE_LIST
                    marker.action = marker.ADD

                    marker.points.append(first_point)
                    marker.points.append(last_point)

                    marker.color.r = 1
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5

                    marker.scale.x = 0.01
                    marker.scale.y = 0
                    marker.scale.z = 0

                    marker.lifetime = rospy.Duration(0.05)

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
