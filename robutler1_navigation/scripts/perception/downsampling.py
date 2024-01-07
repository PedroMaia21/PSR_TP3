#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorSegmentationNode:
    def __init__(self):
        rospy.init_node('downsizing_node', anonymous=False)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to the camera feed
        rospy.Subscriber('/camera2/rgb_rotating/image_raw', Image, self.camera_callback)

        # Publisher for downsampled image
        self.downsampled_pub = rospy.Publisher('/downsampled_image', Image, queue_size=1)

    def camera_callback(self, camera_image_msg):
        try:
            # Convert camera image message to OpenCV format
            camera_image = self.bridge.imgmsg_to_cv2(camera_image_msg, 'bgr8')

            # Downsample the image (adjust the scale factor based on your requirements)
            scale_factor = 0.5
            downsampled_image = cv2.resize(camera_image, None, fx=scale_factor, fy=scale_factor)

            # Convert the downsampled image back to ROS format and publish
            downsampled_image_msg = self.bridge.cv2_to_imgmsg(downsampled_image, 'bgr8')
            self.downsampled_pub.publish(downsampled_image_msg)

        except Exception as e:
            rospy.logerr(f"Error processing camera image: {str(e)}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        segmentation_node = ColorSegmentationNode()
        segmentation_node.run()
    except rospy.ROSInterruptException:
        pass
