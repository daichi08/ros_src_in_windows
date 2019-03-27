#! /usr/bin/env python
# coding: utf-8

import rospy
import math
import numpy as np
from std_msgs.msg    import Header
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

obstacles = []

# 点の座標を定義するフレームの名前
HEADER = Header(frame_id='/laser')

# PointCloud2のフィールドの一覧
FIELDS = [
    # 点の座標(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=0, datatype=PointField.FLOAT32, count=1),
    # 点の色(RGB)
    # 赤: 0xff0000, 緑:0x00ff00, 青: 0x0000ff
    # PointField(name='rgb', offset=0, datatype=PointField.UINT32, count=1),
    ]

def callback(msg):
    global obstacles
    obstacles = []
    lines_point = []
    before_vector = msg.ranges[0]
    angle_inc = msg.angle_increment

    for i in range(len(msg.ranges)):
        current_vector = msg.ranges[i]
        current_point = [current_vector * math.cos(i * angle_inc), current_vector * math.sin(i * angle_inc), 0]
        norm = abs(current_vector - before_vector)
        similarity = 1/(1+norm)

        if similarity > 0.9:
            lines_point.append(current_point)
        else:
            obstacles.append(lines_point)
            lines_point = [current_point]
        before_vector = current_vector
    #for obstacle in obstacles:
        #print(obstacle)

if __name__ == '__main__':
    rospy.init_node("scan_values")
    r = rospy.Rate(1)
    sub = rospy.Subscriber("/scan", LaserScan, callback)
    pub = rospy.Publisher("/outline", PointCloud2, queue_size=1000)

    while not rospy.is_shutdown():
    #for i in range(1):
        if obstacles:
            point_cloud = pc2.create_cloud(HEADER, FIELDS, obstacles[3])
            pub.publish(point_cloud)
        r.sleep()
