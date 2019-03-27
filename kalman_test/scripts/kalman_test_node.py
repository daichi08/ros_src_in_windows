#! /usr/bin/env python
# coding: utf-8

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan

obstacles = []

def callback(msg):
    global obstacles
    obstacles = []
    lines_point = []
    before_vector = msg.ranges[0]

    for i in range(len(msg.ranges)):
        current_vector = msg.ranges[i]
        norm = abs(current_vector - before_vector)
        similarity = 1/(1+norm)
        if similarity > 0.9:
            current_point = [current_vector * math.cos(i*msg.angle_increment), current_vector * math.sin(i*msg.angle_increment)]
            lines_point.append(current_point)
        elif lines_point:
            obstacles.append(lines_point)
            lines_point = []
        before_vector = current_vector

    for obstacle in obstacles:
        print(obstacle)

if __name__ == '__main__':
    rospy.init_node("scan_values")
    r = rospy.Rate(20)
    sub = rospy.Subscriber("/scan", LaserScan, callback)

    #while not rospy.is_shutdown():
    for i in range(4):
        r.sleep()
