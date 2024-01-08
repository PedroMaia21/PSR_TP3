#!/usr/bin/env python3

import rospy
import cv2
import keyboard
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os

# Image processing and capturing function
class CameraNode:

    # -------------------------------
    # Initialization
    # -------------------------------

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/waffle_pi/camera/image_raw", Image, self.camera_callback)

        # The os.path.expanduser expands the tilde in the path
        self.image_dir = os.path.join(os.path.expanduser('~/catkin_ws/src/psr_trabalhofinal/PSR_TP3/Images'))

        # Captures an image when 'p' is pressed
        keyboard.on_press_key('p', self.capture_image)

    # -------------------------------
    # Image Functions
    # -------------------------------

    # Callback function to convert the ROS Image to an OpenCV image and display it
    def camera_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        cv2.imshow("Robot Camera", self.cv_image)
        cv2.waitKey(1)

    # Callback function to capture an image when the 'p' key is pressed
    def capture_image(self, event):
        # The filename is created based on the timestamp and node name
        filename = os.path.join(self.image_dir, rospy.get_name() + "_image_" + str(rospy.get_rostime().to_sec()) + ".jpg")

        # Saves the image
        cv2.imwrite(filename, self.cv_image)
        print("Image saved:", filename)

# -------------------------------
# MAIN
# -------------------------------

# Program execution starts here
if __name__ == '__main__':
    rospy.init_node("photo")
    node = CameraNode()

    # Keeps the program running
    rospy.spin()