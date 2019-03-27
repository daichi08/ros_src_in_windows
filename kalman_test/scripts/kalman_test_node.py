#! /usr/bin/env python
# coding: utf-8

import rospy
import math
from sensor_msgs.msg import LaserScan

obs = []

def callback(msg):
    global obs
    obs = []
    lines_point = []
    datasize = len(msg.ranges)
    offset = -90 * math.pi/180
    before_point = [0, 0]

    for i in range(datasize):
        pass
    print(msg)

if __name__ == '__main__':
    rospy.init_node("scan_values")
    r = rospy.Rate(20)
    sub = rospy.Subscriber("/scan", LaserScan, callback)

    while not rospy.is_shutdown():
        print(obs)
        r.sleep()
