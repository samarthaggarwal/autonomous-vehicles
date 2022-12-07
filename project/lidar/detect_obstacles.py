import rospy
from lidarProcessing import LidarProcessing
from process_birds_eye_view import ProcessBirdsEyeView
from config import *


def run_lidar():
    # init rospy node
    rospy.init_node("lidar_obstacle_map_generation")
    lidarProcessor = LidarProcessing(resolution=resolution, side_range=side_range, fwd_range=fwd_range,
                                     height_range=height_range, is_real_mode=is_real_mode)
    birdsEyeViewProcessor = ProcessBirdsEyeView(resolution=resolution)

    rate = rospy.Rate(3)  # 100 Hz
    lidarProcessor.publish_birds_eye_view()

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        # update birds eye view
        lidarProcessor.publish_birds_eye_view()
    rospy.spin()


if __name__ == "__main__":
    run_lidar()
