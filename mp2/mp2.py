#!/usr/bin/env python3


import time
import math
import numpy as np
import cv2
import rospy
import torch

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
# from skimage import morphology

from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd


class Pedestrian_Detector:
    def __init__(self, model):

        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber("zed2/zed_node/rgb_raw/image_raw_color", Image, self.update_image)
        self.pub_image = rospy.Publisher("pedestrian_detection/annotate", Image, queue_size=1)
        self.detected = False
        self.model = model
        self.rate = rospy.Rate(10)
        self.image = np.ndarray((640, 640))

        # GEM vehicle brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = True
        self.brake_cmd.clear  = False
        self.brake_cmd.ignore = False
        self.brake_cmd.f64_cmd = 0.0
        self.brake_pub.publish(self.brake_cmd)
        

        # GEM vehicle forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = True
        self.accel_cmd.clear  = False
        self.accel_cmd.ignore = False
        self.accel_cmd.f64_cmd = 0.0
        self.accel_pub.publish(self.accel_cmd)


    def img_callback(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        raw_img = cv_image.copy()
        annotated_image = self.detection(raw_img)

        if annotated_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')

            # Publish image message in ROS
            # self.pub_image.publish(out_img_msg)

    def getWorldPedPos(self):
        if self.world_p is not None:
            return self.world_p
        return None

    def update_image(self, data):
        # print("updated image")
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image = cv_image.copy()

    def detection(self):
        while not rospy.is_shutdown():
            image = self.image
            image = cv2.resize(image, (640, 640))
            results = self.model(image)
            
            # add bounding box
            detected_results = results.pandas().xyxy[0]
            humanDetected = False
            for ind,row in detected_results.iterrows():
                if row["class"]==0 and row["confidence"]>0.5:
                    print("person")
                    image = cv2.rectangle(image, (int(row['xmin']), int(row['ymin'])), (int(row['xmax']), int(row['ymax'])), (255, 0, 0), 2)
                    humanDetected = True

            # stopping logic
            xyxy = results.pandas().xyxy[0]
            persons = xyxy[ xyxy.name == "person"]

            if humanDetected:
                self.brake_cmd.f64_cmd = 0.5
                self.brake_pub.publish(self.brake_cmd)
                
                self.accel_cmd.f64_cmd = 0.0
                self.accel_pub.publish(self.accel_cmd)
                print("Brake Engaged!")
            else:
                self.brake_cmd.f64_cmd = 0.0
                self.brake_pub.publish(self.brake_cmd)

                self.accel_cmd.f64_cmd = 0.4
                self.accel_pub.publish(self.accel_cmd)
                print("Gas Engaged!")

            imS = cv2.resize(image, (1920, 1080))
            cv2.imshow("output", imS)
            cv2.waitKey(100)

            self.rate.sleep()

        return image

if __name__ == '__main__':
    # rospy.init_node('lanenet_node', anonymous=True)
    # pd = Pedestrian_Detector(model)

    rospy.init_node('sos_node', anonymous=True)
    model = torch.hub.load(r'./yolov5', 'custom', path=r'./yolov5s.pt', source = 'local')
    pd = Pedestrian_Detector(model)
    pd.detection()

    while not rospy.is_shutdown():
        pd.rate.sleep()