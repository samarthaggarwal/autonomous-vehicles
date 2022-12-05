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
from lidar.util import transform_to_image_coordinates, transform_to_taped_coordinates
from lidar.config import resolution


class Summon(object):

    def __init__(self):

        self.rate = rospy.Rate(100)

        self.look_ahead = 6  # meters
        self.wheelbase = 1.75  # meters
        self.goal = 0

        self.read_waypoints()  # read waypoints
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

        self.destination = (50.0, -10.0)  # in metric coordinate system with taped start as origin
        # convert to image coordinate system
        self.destinationImageCoordinates = transform_to_image_coordinates(self.destination[0], self.destination[1],
                                                                          resolution)

    def updateObstacleMap(self, data):
        """
        update obstacle map on every event
        """
        self.obstacleMap = self.cvBridge.imgmsg_to_cv2(data, 'mono8').astype(np.uint8) // 255
        # print(self.obstacleMap)

    # import waypoints.csv into a list (path_points)
    def update_gps(self, gps_data):
        # print("updated GPS")
        self.curr_lat = gps_data.latitude
        self.curr_lon = gps_data.longitude

    def read_waypoints(self):

        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '/home/akash/Documents/autonomous-vehicles/mp4/oval.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # print(path_points)

        # turn path_points into a list of floats to eliminate the need for casts
        self.path_points_x = [float(point[0]) for point in path_points]
        self.path_points_y = [float(point[1]) for point in path_points]
        self.path_points_yaw = [self.heading_to_yaw(float(point[2])) for point in path_points]
        self.dist_arr = np.zeros(len(self.path_points_x))

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

    def plot_path(self, path, obstacleMapRGB):
        """
        plots path on RVIZ
        """
        if self.obstacleMap is None or len(path)<2:
            return
        # obstacleMapRGB = cv2.cvtColor(self.obstacleMap * 255, cv2.COLOR_GRAY2RGB)
        color = (0, 255, 0)
        for i in range(len(path)-1):
            obstacleMapRGB = cv2.line(obstacleMapRGB, path[i][::-1], path[i+1][::-1], color, 5)
            obstacleMapRGB = cv2.circle(obstacleMapRGB, path[i+1][::-1], 10, (0, 255, 255), -1)

        obstacleMapRGB = cv2.circle(obstacleMapRGB, path[0][::-1], 10, (255, 0, 0), -1)
        obstacleMapRGB = cv2.circle(obstacleMapRGB, path[-1][::-1], 10, (0, 0, 255), -1)

        obstacleMapRGB = obstacleMapRGB.astype(np.uint8)
        # cv2.imshow("Voronoi Path", obstacleMapRGB)
        # cv2.waitKey(1)
        obstacleMapRGB = self.cvBridge.cv2_to_imgmsg(obstacleMapRGB, '8UC3')
        self.voronoiMapPub.publish(obstacleMapRGB)

    def start(self):
        time.sleep(1)
        while not rospy.is_shutdown():
            # print("loop")
            # startTime = time.time()
            # get current position and orientation in the world frame
            curr_x, curr_y, curr_yaw = self.get_gem_pose()
            print(curr_x, curr_y, curr_yaw)
            print(self.curr_lat, self.curr_lon)
            lon_wp_x, lat_wp_y = axy.ll2xy(self.curr_lat, self.curr_lon, self.olat, self.olon)

            origin_shift_x = curr_x - lon_wp_x
            origin_shift_y = curr_y - lat_wp_y
            print("Lon wp x: ", lon_wp_x)
            print("Lat wp y: ", lat_wp_y)
            print("Curr Yaw Value: ", curr_yaw * 180.0 / np.pi)

            srcImage = transform_to_image_coordinates(lon_wp_x, lat_wp_y, resolution)

            while self.obstacleMap is None:
                self.rate.sleep()
            # beforeVoronoi = time.time()
            # print(f"before Voronoi = {beforeVoronoi - startTime}")
            print(f"Obstacle Map Shape - {self.obstacleMap.shape}")
            voronoi = Voronoi(self.obstacleMap)
            # afterInit = time.time()
            # print(f"after Init = {afterInit - beforeVoronoi}")
            path = voronoi.path((srcImage[1], srcImage[0]), (self.destinationImageCoordinates[1], self.destinationImageCoordinates[0]))
            # afterPath = time.time()
            # print(f"after Path = {afterPath - afterInit}")
            if len(path)==0:
                raise Exception("No path exists")
            print(f"path: {path}")
            voronoi.visualise_path(path)
            self.plot_path(path, voronoi.plot_regions())
            nextPointImage = path[1]
            # nextPointImage = (0, 0)

            nextPointRealX, nextPointRealY = transform_to_taped_coordinates(nextPointImage[1], nextPointImage[0])

            nextPointRealX += origin_shift_x
            nextPointRealY += origin_shift_y

            # finding the distance of each way point from the current position
            distance = self.dist((nextPointRealX, nextPointRealY), (curr_x, curr_y))

            # finding the distance between the goal point and the vehicle
            # true look-ahead distance between a waypoint and current position
            L = distance

            print(f"goal: {self.goal}, L:{L}")

            # transforming the goal point into the vehicle coordinate frame
            gvcx = nextPointRealX - curr_x
            gvcy = nextPointRealY - curr_y
            goal_x_veh_coord = gvcx * np.cos(curr_yaw) + gvcy * np.sin(curr_yaw)
            goal_y_veh_coord = gvcy * np.cos(curr_yaw) - gvcx * np.sin(curr_yaw)

            # find the curvature and the angle
            alpha = self.path_points_yaw[self.goal] - (curr_yaw)
            k = 0.285
            angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / L)
            angle = angle_i * 2
            angle = round(np.clip(angle, -0.61, 0.61), 3)

            ct_error = round(np.sin(alpha) * L, 3)

            print("Crosstrack Error: " + str(ct_error))

            # implement constant pure pursuit controller
            self.ackermann_msg.speed = 2.8
            self.ackermann_msg.steering_angle = angle
            # self.ackermann_pub.publish(self.ackermann_msg)

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
