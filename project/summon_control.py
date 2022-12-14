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
import config

# ROS Headers
import alvinxy.alvinxy as axy  # Import AlvinXY transformation module
import rospy

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

def transform_points_from_image_to_tape(points, resolution):
    path = []
    for pathInd in range(len(points)):
        nextChasePointXTape, nextChasePointYTape = transform_to_taped_coordinates(points[pathInd][1],
                                                                                  points[pathInd][0],
                                                                                  resolution)
        path.append([nextChasePointXTape, nextChasePointYTape])
    path = np.array(path)
    return path

class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm = 0
        self.last_t = None
        self.last_e = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wg = wg
        self.derror = 0

    def reset(self):
        self.iterm = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de

    def set_control(self):
        """
        runs a while loop and gives the control outputs to the vehicle
        """
        pass


class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)

    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted


class PurePursuit(object):

    def __init__(self):

        self.rate = rospy.Rate(50)

        self.look_ahead = 5
        self.far_look_ahead = 8  # meters
        self.wheelbase = 1.75  # meters
        self.offset = 0.46  # meters

        self.gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.lat = None
        self.lon = None
        self.heading = 0.0
        self.cvBridge = CvBridge()

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.speed = 0.0

        self.bezierPathTapeCoordinates = None
        self.beizerPathSub = rospy.Subscriber("/mp0/BeizerPath", Image, self.beizer_path_update)

        self.olat = 40.0928563
        self.olon = -88.2359994

        self.desired_speed = 0.4  # m/s, reference speed
        # self.max_accel = 0.4  # % of acceleration
        self.pid_speed = PID(1.2, 0.2, 0.6, wg=20)
        self.speed_filter = OnlineFilter(1.2, 30, 4)

        # -------------------- PACMod setup --------------------

        self.gem_enable = False
        self.pacmod_enable = False

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2  # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1  # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0  # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0  # radians/second

        # path planner subscriptions
        # subscribe to obstacle map from Lidar
        self.destinationTapeCoordinates = config.destinationTapeCoordinates  # in metric coordinate system with taped start as origin
        # convert to image coordinate system
        self.destinationImageCoordinates = transform_to_image_coordinates(self.destinationTapeCoordinates[0],
                                                                          self.destinationTapeCoordinates[1],
                                                                          resolution)

    def beizer_path_update(self, data):
        bezierImageCoordinates = self.cvBridge.imgmsg_to_cv2(data, 'mono8').astype(np.uint8)
        self.bezierPathTapeCoordinates = transform_points_from_image_to_tape(bezierImageCoordinates, resolution)

    def inspva_callback(self, inspva_msg):
        self.lat = inspva_msg.latitude  # latitude
        self.lon = inspva_msg.longitude  # longitude
        self.heading = inspva_msg.azimuth  # heading in degrees

    def speed_callback(self, msg):
        self.speed = round(msg.data, 3)  # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

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

    def front2steer(self, f_angle):

        if (f_angle > 35):
            f_angle = 35

        if (f_angle < -35):
            f_angle = -35

        if (f_angle > 0):
            steer_angle = round(-0.1084 * f_angle ** 2 + 21.775 * f_angle, 2)

        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084 * f_angle ** 2 + 21.775 * f_angle, 2)
        else:
            steer_angle = 0.0

        return steer_angle

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

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def start_pp(self):

        while not rospy.is_shutdown():

            if (self.gem_enable == False):

                if (self.pacmod_enable == True):
                    # ---------- enable PACMod ----------

                    # enable forward gear
                    self.gear_cmd.ui16_cmd = 3

                    # enable brake
                    self.brake_cmd.enable = True
                    self.brake_cmd.clear = False
                    self.brake_cmd.ignore = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable = True
                    self.accel_cmd.clear = False
                    self.accel_cmd.ignore = False
                    self.accel_cmd.f64_cmd = 0.0

                    self.gear_pub.publish(self.gear_cmd)
                    print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    print("Turn Signal Ready!")

                    self.brake_pub.publish(self.brake_cmd)
                    print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    print("Gas Engaged!")

                    self.gem_enable = True

            while self.bezierPathTapeCoordinates is None:
                self.rate.sleep()

            curr_x, curr_y, curr_yaw = self.get_gem_state()
            # print(f"curr_x: {curr_x}, curr_y: {curr_y}")

            currLocationImageCoordinates = transform_to_image_coordinates(curr_x, curr_y, resolution)

            dist_arr = []

            # finding the distance of each way point from the current position
            for i in range(len(self.bezierPathTapeCoordinates)):
                p_dist = self.dist((self.bezierPathTapeCoordinates[i][0], self.bezierPathTapeCoordinates[i][1]),
                                   (curr_x, curr_y))
                dist_arr.append(p_dist)

            """ Find nearest goal point in each iteration """
            goal = 0
            # print(f"length of dist_arr: {len(dist_arr)} | dist[goal]: {dist_arr[goal]} | self.look_ahead: {self.look_ahead} | look ahead check: {dist_arr[goal] < self.look_ahead - 0.3}")
            while goal < len(dist_arr) and dist_arr[goal] < self.look_ahead - 0.3:
                goal += 1
            far_ahead_point = goal
            while far_ahead_point < len(dist_arr) and dist_arr[far_ahead_point] < self.far_look_ahead - 0.3:
                far_ahead_point += 1

            goal = min(goal, len(dist_arr) - 1)
            far_ahead_point = min(far_ahead_point, len(dist_arr) - 1)
            distance_from_next_chase_point = dist_arr[goal]

            print(f"goal: {goal}, distance_from_next_chase_point:{distance_from_next_chase_point}")

            distance_from_destination = self.dist(self.destinationTapeCoordinates, (curr_x, curr_y))

            def orientation_ray_from_vehicle(point):
                nextChasePointXTape, nextChasePointYTape = self.bezierPathTapeCoordinates[point][0], \
                    self.bezierPathTapeCoordinates[point][1]
                # transforming the goal point into the vehicle coordinate frame
                gvcx = nextChasePointXTape - curr_x
                gvcy = nextChasePointYTape - curr_y
                orientation_degrees = ((np.arctan2(gvcx, gvcy) * 180 / np.pi) + 360.0) % 360.0
                orientation = self.heading_to_yaw(orientation_degrees)
                return orientation

            orientationOfLineConnectingNextChasePoint = orientation_ray_from_vehicle(goal)
            orientationOfLineConnectingFarLookAheadPoint = orientation_ray_from_vehicle(far_ahead_point)

            # find the curvature and the angle
            alpha = orientationOfLineConnectingNextChasePoint - curr_yaw
            alpha_far = orientationOfLineConnectingFarLookAheadPoint - curr_yaw

            # ----------------- tuning this part as needed -----------------
            k = 0.41
            angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / distance_from_next_chase_point)
            angle = angle_i * 2
            # ----------------- tuning this part as needed -----------------

            f_delta = round(np.clip(angle, -0.61, 0.61), 3)

            f_delta_deg = np.degrees(f_delta)

            # steering_angle in degrees
            steering_angle = self.front2steer(f_delta_deg)

            if (self.gem_enable == True):
                print("Current index: " + str(goal))
                print("Forward velocity: " + str(self.speed))
                ct_error = round(np.sin(alpha) * distance_from_next_chase_point, 3)
                print("Crosstrack Error: " + str(ct_error))
                print("Front steering angle: " + str(np.degrees(f_delta)) + " degrees")
                print("Steering wheel angle: " + str(steering_angle) + " degrees")
                print("\n")

            # implement constant pure pursuit controller
            if distance_from_destination < 4.0:
                self.brake_cmd.f64_cmd = 0.5
                self.brake_pub.publish(self.brake_cmd)
                print("Destination Reached!!!!!!!!!!!!!!!!!!!!!!!!!!")
                break

            angularThreshold = np.pi / 18
            if np.abs(alpha) > angularThreshold or np.abs(alpha_far) > angularThreshold:
                desired_speed = self.desired_speed / 2
            else:
                desired_speed = self.desired_speed

            current_time = rospy.get_time()
            filt_vel = self.speed_filter.get_data(self.speed)
            print(f"filtered speed: {filt_vel}")
            # output_accel = self.pid_speed.get_control(current_time, desired_speed - filt_vel)
            output_accel = self.pid_speed.get_control(current_time, desired_speed - self.speed)
            print("Output Acceleration - ", output_accel)
            print(f"speed: {self.speed}")

            min_speed = 0.1
            max_speed = 1.0
            stopped_speed = 0.05
            speed_tolerance = 0.2
            max_accel = 0.4
            accel_for_const_speed = 0.315

            if filt_vel < stopped_speed:
                min_accel = 0.35
            else:
                min_accel = 0.1 # to be decided

            if filt_vel > desired_speed: 
                output_accel = accel_for_const_speed

            output_accel = min(max(output_accel, min_accel), max_accel)

            if f_delta_deg <= 30 and f_delta_deg >= -30:
                self.turn_cmd.ui16_cmd = 1
            elif f_delta_deg > 30:
                self.turn_cmd.ui16_cmd = 2  # turn left
            else:
                self.turn_cmd.ui16_cmd = 0  # turn right

            # if output_accel > 0:
            self.accel_cmd.f64_cmd = output_accel
            # print(self.accel_cmd)
            self.accel_pub.publish(self.accel_cmd)
            # print("Accel Done")
            self.steer_cmd.angular_position = np.radians(steering_angle)
            self.steer_pub.publish(self.steer_cmd)
            self.turn_pub.publish(self.turn_cmd)

            self.rate.sleep()


def pure_pursuit():
    rospy.init_node('gnss_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()
