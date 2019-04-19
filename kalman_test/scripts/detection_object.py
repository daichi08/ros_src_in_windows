#! /usr/bin/env python
# coding: utf-8

import rospy
import math
import numpy as np
import gridmap as gm
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

points_x = []
points_y = []

def callback(msg):
    global points_x
    global points_y

    index    = 0
    datasize = len(msg.ranges)
    rad_inc  = msg.angle_increment
    rad_min  = msg.angle_min
    points_x = []
    points_y = []

    for range_ in msg.ranges:
        if not math.isnan(range_) and range_ != 0:
            angle = rad_min + rad_inc * index
            point_x = range_ * math.cos(angle)
            point_y = range_ * math.sin(angle)
            points_x.append(point_x)
            points_y.append(point_y)
        index += 1

def main():
    rospy.init_node("scan_values")
    r   = rospy.Rate(20)
    sub = rospy.Subscriber("/scan", LaserScan, callback, queue_size=1)

    while not rospy.is_shutdown():
        if points_x:
            print("ok")
            grid_map = gm.main(points_x, points_y)
        else:
            print("wait")
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
