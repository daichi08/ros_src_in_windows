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

def circle_fitting(x, y):
    sumx  = sum(x)
    sumy  = sum(y)
    sumx2 = sum([x_i ** 2 for x_i in x])
    sumy2 = sum([y_i ** 2 for y_i in y])
    sumxy = sum([x_i * y_i for (x_i, y_i) in zip(x, y)])

    F = np.array([[sumx2,sumxy,sumx],
                  [sumxy,sumy2,sumy],
                  [sumx,sumy,len(x)]])

    G = np.array([[-sum([x_i ** 3 + x_i*y_i **2 for (x_i, y_i) in zip(x, y)])],
                  [-sum([x_i ** 2 *y_i + y_i **3 for (x_i ,y_i) in zip(x, y)])],
                  [-sum([x_i ** 2 + y_i **2 for (x_i, y_i) in zip(x, y)])]])

    T=np.linalg.inv(F).dot(G)

    cxe=float(T[0]/-2)
    cye=float(T[1]/-2)
    re=math.sqrt(cxe**2+cye**2-T[2])
    #print (cxe,cye,re)
    return cxe,cye,re


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
                if len(obstacle) > 2:
                    x = list(np.array(obstacle)[:,0])
                    y = list(np.array(obstacle)[:,1])
                    c_x, c_y, c_r = circle_fitting(x, y)

                    # 以下Rviz
                    marker = Marker()

                    marker.header.frame_id = "laser"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "obstacles"
                    marker.id = index

                    marker.type   = marker.CYLINDER
                    marker.action = marker.ADD

                    marker.pose.position.x = c_x
                    marker.pose.position.y = c_y
                    marker.pose.position.z = 0

                    marker.scale.x = c_r
                    marker.scale.y = c_r
                    marker.scale.z = 0.1

                    marker.color.a = 0.6
                    marker.color.r = 0
                    marker.color.g = 1
                    marker.color.b = 0

                    marker.lifetime = rospy.Duration(0.1)

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
