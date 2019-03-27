#! /usr/bin/env python
# coding: utf-8

import rospy
import math
from sensor_msgs.msg import LaserScan

obs = []

def callback(msg):
    global obs
    points = msg.ranges
    before_point = msg.ranges[0]
    obs = []
    lines_point = []

    for point in points:
        norm = abs(before_point - point)
        lim_norm = math.sin(0.36*math.pi/180)*3
        if norm < lim_norm:
            lines_point.append(point)
        else:
            if len(lines_point) > 0:
                obs.append(lines_point)
                lines_point = []
        before_point = point

if __name__ == '__main__':
    rospy.init_node("scan_values")
    r = rospy.Rate(20)
    sub = rospy.Subscriber("/scan", LaserScan, callback)

    while not rospy.is_shutdown():
        print(obs)
        r.sleep()
