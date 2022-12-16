import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ProcessBirdsEyeView:
    def __init__(self, resolution):
        self.resolution = resolution

        self.birdsEyeSub = rospy.Subscriber("/mp0/BirdsEye", Image, self.processBirdsEyeView, queue_size=1)
        self.obstacleMapPub = rospy.Publisher("/mp0/ObstacleMap", Image, queue_size=1)

        # self.position = 0

        self.cvBridge = CvBridge()
        self.aggregated_img = None
        self.finalIm = None

    def processBirdsEyeView(self, data):
        """
            Uses Hough circles to detect the location of the pedestrian in the
            birds eye view image and saves it.

            Input: data - the birds eye view Lidar image

            Output: None

            Side Effects: Changes the detected position of the pedestrian
        """
        # Use Hough Circle detector to detect pedestrian
        # Reference: https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html

        img = self.cvBridge.imgmsg_to_cv2(data, 'mono8').astype(np.uint8)
        x_max = img.shape[1]
        y_max = img.shape[0]

        processedImg = self.preprocessImg(img)
        if self.aggregated_img is None:
            self.aggregated_img = processedImg
            self.aggregated_img[0, :] = 255
            self.aggregated_img[-1, :] = 255
        else:
            self.aggregated_img = self.aggregated_img + processedImg
            self.aggregated_img = np.where(self.aggregated_img > 0, 255, 0).astype(np.uint8)

        np.save("voronoiObstacleMap.npy", self.aggregated_img)

        self.obstacleMapPub.publish(self.cvBridge.cv2_to_imgmsg(self.aggregated_img, 'mono8'))

    def preprocessImg(self, img):
        """
            Preprocesses the birds eye view lidar image so that we have a better
            image for circle detection.

            Input: img - birds eye view lidar image

            Output: processed image
        """
        kernel = np.array([[0, 1, 0],
                           [0, 1, 0],
                           [0, 1, 0]], np.uint8)

        # dilate to thicken lines
        dilated_img = cv2.dilate(img, kernel, iterations=8)

        # erode to smoothen the lines
        eroded_img = cv2.erode(dilated_img, kernel, iterations=8)

        # blur to enchance regions
        blurred_img = cv2.GaussianBlur(eroded_img, (5, 5), 9)

        # threshold so that any regions that matter are accounted for
        _, threshold_img = cv2.threshold(blurred_img, 1, 255, cv2.THRESH_BINARY)

        return threshold_img
