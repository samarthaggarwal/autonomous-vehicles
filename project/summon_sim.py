#!/usr/bin/env python3

# Python Headers
import os
import csv
import math
import time
import cv2

import numpy as np
from numpy import linalg as la

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, NavSatFix, Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
import alvinxy.alvinxy as axy

from voronoi.voronoi import Voronoi
from cv_bridge import CvBridge
import numpy as np
from lidar.util import *
from lidar.config import resolution


def heading_to_yaw(heading_curr):
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


class Summon(object):

    def __init__(self):

        self.rate = rospy.Rate(100)

        self.look_ahead = 6  # meters
        self.wheelbase = 1.75  # meters
        self.goal = 0

        self.olat, self.olon = 40.0928563, -88.2359994

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration = 0.0
        self.ackermann_msg.jerk = 0.0
        self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle = 0.0
        self.curr_lat = 0.0
        self.curr_lon = 0.0

        self.ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.update_gps, queue_size=1)

        # subscribe to obstacle map from Lidar
        self.obstacleMap = None
        self.obstacleMapSub = rospy.Subscriber("/mp0/ObstacleMap", Image, self.updateObstacleMap, queue_size=1)
        self.voronoiMapPub = rospy.Publisher("/mp0/VoronoiPath", Image, queue_size=1)

        self.cvBridge = CvBridge()

        self.destinationTapeCoordinates = (50.0, -10.0)  # in metric coordinate system with taped start as origin
        # convert to image coordinate system
        self.destinationImageCoordinates = transform_to_image_coordinates(self.destinationTapeCoordinates[0],
                                                                          self.destinationTapeCoordinates[1],
                                                                          resolution)

    def updateObstacleMap(self, data):
        """
        update obstacle map on every event
        """
        self.obstacleMap = self.cvBridge.imgmsg_to_cv2(data, 'mono8').astype(np.uint8) // 255

    def update_gps(self, gps_data):
        self.curr_lat = gps_data.latitude
        self.curr_lon = gps_data.longitude

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    def get_gem_pose(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            service_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = service_response(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q = model_state.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        return round(x, 4), round(y, 4), round(yaw, 4)

    def plot_path(self, path, obstacleMapRGB):
        """
        plots path on RVIZ
        """
        if self.obstacleMap is None or len(path) < 2:
            return
        color = (0, 255, 0)
        for i in range(len(path) - 1):
            obstacleMapRGB = cv2.line(obstacleMapRGB, path[i][::-1], path[i + 1][::-1], color, 5)
            obstacleMapRGB = cv2.circle(obstacleMapRGB, path[i + 1][::-1], 3, (0, 255, 255), -1)
            obstacleMapRGB = cv2.circle(obstacleMapRGB, path[i][::-1], 3, (0, 255, 255), -1)

        obstacleMapRGB = cv2.circle(obstacleMapRGB, path[0][::-1], 3, (255, 0, 0), -1)
        obstacleMapRGB = cv2.circle(obstacleMapRGB, path[-1][::-1], 3, (0, 0, 255), -1)

        obstacleMapRGB = obstacleMapRGB.astype(np.uint8)
        # cv2.imshow("Voronoi Path", obstacleMapRGB)
        # cv2.waitKey(1)
        obstacleMapRGB = self.cvBridge.cv2_to_imgmsg(obstacleMapRGB, '8UC3')
        self.voronoiMapPub.publish(obstacleMapRGB)

    def start(self):
        time.sleep(1)
        while not rospy.is_shutdown():
            # startTime = time.time()
            # get current position and orientation in the world frame or simulator frame
            curr_x_simulator, curr_y_simulator, curr_yaw_simulator = self.get_gem_pose()
            # get the current position in taped origin coordinate frame
            curr_x_tape, curr_y_tape = axy.ll2xy(self.curr_lat, self.curr_lon, self.olat, self.olon)
            currLocationImageCoordinates = transform_to_image_coordinates(curr_x_tape, curr_y_tape, resolution)

            while self.obstacleMap is None:
                self.rate.sleep()
            # beforeVoronoi = time.time()
            # print(f"before Voronoi = {beforeVoronoi - startTime}")
            voronoi = Voronoi(self.obstacleMap)
            # afterInit = time.time()
            # print(f"after Init = {afterInit - beforeVoronoi}")
            try:
                path = voronoi.path(currLocationImageCoordinates[::-1], self.destinationImageCoordinates[::-1])
            except Exception as e:
                print(f"ERROR: {str(e)}")
                continue
            # afterPath = time.time()
            # print(f"after Path = {afterPath - afterInit}")

            self.plot_path(path, voronoi.plot_regions())
            nextChasePointImageCoordinates = path[1][::-1]
            nextChasePointXTape, nextChasePointYTape = transform_to_taped_coordinates(nextChasePointImageCoordinates[0],
                                                                                      nextChasePointImageCoordinates[1],
                                                                                      resolution)
            nextChasePointXSimulator, nextChasePointYSimulator = transform_tape_to_simulator(nextChasePointXTape,
                                                                                             nextChasePointYTape)

            distance_from_next_chase_point = self.dist((nextChasePointXSimulator, nextChasePointYSimulator),
                                                       (curr_x_simulator, curr_y_simulator))

            if distance_from_next_chase_point > self.look_ahead:
                t = self.look_ahead / distance_from_next_chase_point
                nextChasePointXSimulator = (1.0 - t) * curr_x_simulator + t * nextChasePointXSimulator
                nextChasePointYSimulator = (1.0 - t) * curr_y_simulator + t * nextChasePointYSimulator
                distance_from_next_chase_point = self.dist((nextChasePointXSimulator, nextChasePointYSimulator),
                                                           (curr_x_simulator, curr_y_simulator))

            distance_from_destination = self.dist(self.destinationTapeCoordinates, (curr_x_tape, curr_y_tape))

            # transforming the goal point into the vehicle coordinate frame
            gvcx = nextChasePointXSimulator - curr_x_simulator
            gvcy = nextChasePointYSimulator - curr_y_simulator
            goal_x_veh_coord = gvcx * np.cos(curr_yaw_simulator) + gvcy * np.sin(curr_yaw_simulator)
            goal_y_veh_coord = gvcy * np.cos(curr_yaw_simulator) - gvcx * np.sin(curr_yaw_simulator)

            orientationOfLineConnectingNextChasePoint = np.arctan2(gvcy, gvcx)

            print(f"Curr Yaw Value: {curr_yaw_simulator * 180.0 / np.pi},\n"
                  f"Orientation of Path: {orientationOfLineConnectingNextChasePoint * 180 / np.pi}\n"
                  f"Curr Location in Simulator World - {curr_x_simulator, curr_y_simulator}\n"
                  f"Next Chase Location in Simulator World - {nextChasePointXSimulator, nextChasePointYSimulator}\n")

            # find the curvature and the angle
            alpha = orientationOfLineConnectingNextChasePoint - curr_yaw_simulator
            k = 0.285
            angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / distance_from_next_chase_point)
            angle = angle_i * 2
            angle = round(np.clip(angle, -0.61, 0.61), 3)

            print(f"Alpha Angle - {alpha * 180.0 / np.pi}\n"
                  f"Distance from Next Chase Point - {distance_from_next_chase_point}\n"
                  f"Steering Angle - {angle * 180.0 / np.pi}\n"
                  f"-------------------------------------")

            ct_error = round(np.sin(alpha) * distance_from_next_chase_point, 3)
            # print("Crosstrack Error: " + str(ct_error))

            # implement constant pure pursuit controller
            if distance_from_destination < 4.0:
                self.ackermann_msg.speed = 0.0
                self.ackermann_msg.steering_angle = angle
                self.ackermann_pub.publish(self.ackermann_msg)
                print("Destination Reached!!!!!!!!!!!!!!!!!!!!!!!!!!")
                break

            if distance_from_next_chase_point < 5.0:
                self.ackermann_msg.speed = 1.0
            else:
                self.ackermann_msg.speed = 2.8

            self.ackermann_msg.steering_angle = angle
            self.ackermann_pub.publish(self.ackermann_msg)

            # endTime = time.time()
            # print(f"end = {endTime - afterPath}")

            self.rate.sleep()


def summon():
    rospy.init_node('summon_sim_node', anonymous=True)
    s = Summon()

    try:
        s.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    summon()
