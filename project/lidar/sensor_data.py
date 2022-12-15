#!/usr/bin/env python3

import math
import random
import numpy as np

# ROS Headers
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

import alvinxy.alvinxy as axy


class GpsSensorData(object):

    def __init__(self):
        self.gps_sub = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.rate = rospy.Rate(10)

        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.imu_yaw = 0.0
        self.olat = 40.0928563 # origin latitude
        self.olon = -88.2359994 # origin longitude

    def gps_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)
        self.alt = round(msg.altitude, 6)

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_accel = msg.linear_acceleration
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.imu_yaw = round(yaw, 6) * 57.2957795131  # convert to degrees

    def get_current_gps_sensor(self):
        wp_x, wp_y = axy.ll2xy(self.lat, self.lon, self.olat, self.olon)
        return wp_x, wp_y, self.imu_yaw


class GpsSensorRealData(object):

    def __init__(self):
        self.gps_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.gps_callback)
        self.rate = rospy.Rate(10)
        self.lat = 0.0
        self.lon = 0.0
        self.yaw = 0.0
        self.olat = 40.0928563 # origin latitude
        self.olon = -88.2359994 # origin longitude

    def gps_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)
        self.yaw = - round(msg.azimuth, 6) + 270
        if 270 >= self.yaw >= 180:
            self.yaw = self.yaw - 360.0 

    def get_current_gps_sensor(self):
        wp_x, wp_y = axy.ll2xy(self.lat, self.lon, self.olat, self.olon)
        return wp_x, wp_y, self.yaw
