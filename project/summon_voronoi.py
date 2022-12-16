#!/usr/bin/env python3

# ================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/13/2021                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
# ================================================================

from __future__ import print_function

# Python Headers
import os
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import alvinxy.alvinxy as axy  # Import AlvinXY transformation module
import rospy

from rospy.numpy_msg import numpy_msg

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from sensor_msgs.msg import Image
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

from voronoi.voronoi import Voronoi
from cv_bridge import CvBridge
from lidar.util import transform_to_image_coordinates, transform_to_taped_coordinates
from lidar.config import resolution
import cv2
from smoothpath import bezier_curve
import config


class BeizerPath(object):

    def __init__(self):

        self.rate = rospy.Rate(100)

        self.look_ahead = 6
        self.far_look_ahead = 10  # meters
        self.wheelbase = 1.75  # meters
        self.offset = 0.46  # meters

        self.gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.lat = None
        self.lon = None
        self.heading = 0.0

        self.olat = 40.0928563
        self.olon = -88.2359994

        self.obstacleMap = None
        self.obstacleMapSub = rospy.Subscriber("/mp0/ObstacleMap", Image, self.updateObstacleMap, queue_size=1)
        self.voronoiMapPub = rospy.Publisher("/mp0/VoronoiPath", Image, queue_size=1)
        self.bezierPathPub = rospy.Publisher("/mp0/BeizerPath", Image, queue_size=1)

        self.cvBridge = CvBridge()

        self.destinationTapeCoordinates = config.destinationTapeCoordinates  # in metric coordinate system with taped start as origin
        # convert to image coordinate system
        self.destinationImageCoordinates = transform_to_image_coordinates(self.destinationTapeCoordinates[0],
                                                                          self.destinationTapeCoordinates[1],
                                                                          resolution)

    def plot_path(self, path, obstacleMapRGB):
        """Rate
        plots path on RVIZ
        """
        if self.obstacleMap is None or len(path) < 2:
            return
        # obstacleMapRGB = cv2.cvtColor(self.obstacleMap * 255, cv2.COLOR_GRAY2RGB)
        color = (0, 255, 0)
        for i in range(len(path) - 1):
            obstacleMapRGB = cv2.line(obstacleMapRGB, path[i][::-1], path[i + 1][::-1], color, 5)
            obstacleMapRGB = cv2.circle(obstacleMapRGB, path[i + 1][::-1], 10, (0, 255, 255), -1)

        obstacleMapRGB = cv2.circle(obstacleMapRGB, path[0][::-1], 10, (255, 0, 0), -1)
        obstacleMapRGB = cv2.circle(obstacleMapRGB, path[-1][::-1], 10, (0, 0, 255), -1)

        obstacleMapRGB = obstacleMapRGB.astype(np.uint8)
        # cv2.imshow("Voronoi Path", obstacleMapRGB)
        # cv2.waitKey(1)
        obstacleMapRGB = self.cvBridge.cv2_to_imgmsg(obstacleMapRGB, '8UC3')
        self.voronoiMapPub.publish(obstacleMapRGB)

    def inspva_callback(self, inspva_msg):
        self.lat = inspva_msg.latitude  # latitude
        self.lon = inspva_msg.longitude  # longitude
        self.heading = inspva_msg.azimuth  # heading in degrees

    def updateObstacleMap(self, data):
        """
        update obstacle map on every event
        """
        self.obstacleMap = self.cvBridge.imgmsg_to_cv2(data, 'mono8').astype(np.uint8) // 255
        # self.obstacleMap = np.zeros((151, 401), dtype=np.uint8)
        # self.obstacleMap[0, :] = 1
        # print(self.obstacleMap.shape)

    def heading_to_yaw(self, heading_curr):
        # 0   <= heading < 90  --- 90 to 0     (pi/2 to 0)
        # 90  <= heading < 180 --- 0 to -90    (0 to -pi/2)
        # 180 <= heading < 270 --- -90 to -180 (-pi/2 to -pi)
        # 270 <= heading < 360 --- 180 to 90   (pi to pi/2)
        if (heading_curr >= 0 and heading_curr < 90):
            yaw_curr = np.radians(90 - heading_curr)
        elif (heading_curr >= 90 and heading_curr < 180):
            yaw_curr = np.radians(90 - heading_curr)
        elif (heading_curr >= 180 and heading_curr < 270):
            yaw_curr = np.radians(90 - heading_curr)
        else:
            yaw_curr = np.radians(450 - heading_curr)
        return yaw_curr

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y

    def get_gem_state(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
        # print(f"lat: {self.lat} | lon: {self.lon} | local_x_curr: {local_x_curr} | local_y_curr: {local_y_curr}")

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading)

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)

        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def start_beizer(self):

        while not rospy.is_shutdown():

            while self.lat is None or self.lon is None:
                self.rate.sleep()

            curr_x, curr_y, curr_yaw = self.get_gem_state()
            # print(f"curr_x: {curr_x}, curr_y: {curr_y}")

            currLocationImageCoordinates = transform_to_image_coordinates(curr_x, curr_y, resolution)

            while self.obstacleMap is None:
                self.rate.sleep()

            print(f"Obstacle Map Shape: {self.obstacleMap.shape}")
            voronoi = Voronoi(self.obstacleMap)
            try:
                path = voronoi.path(currLocationImageCoordinates[::-1], self.destinationImageCoordinates[::-1])
                bezierPathImageCoordinates = bezier_curve(np.array(path), 1000, voronoi, resolution)
                bezierPathImageCoordinates = bezierPathImageCoordinates[::-1]
                self.plot_path(bezierPathImageCoordinates, voronoi.plot_regions())
            except Exception as e:
                print(f"src = {currLocationImageCoordinates} destination = {self.destinationImageCoordinates}")
                print(str(e))
                continue

            bezierPathImageCoordinates = np.array(bezierPathImageCoordinates).astype(np.uint8)
            self.bezierPathPub.publish(self.cvBridge.cv2_to_imgmsg(bezierPathImageCoordinates, 'mono8'))
            # self.rate.sleep()


def bezier_publisher():
    rospy.init_node('gnss_beizer_node', anonymous=True)
    pp = BeizerPath()

    try:
        pp.start_beizer()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    bezier_publisher()
