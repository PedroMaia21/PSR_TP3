#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorSegmentationNode:
    def __init__(self):
        rospy.init_node('color_segmentation_node', anonymous=False)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to the YOLO node's output
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.yolo_callback)

        # Publisher for segmented image
        self.segmented_pub = rospy.Publisher('/color_segmentation/segmented_image', Image, queue_size=1)

    def yolo_callback(self, yolo_image_msg):
        try:
            # Convert YOLO image message to OpenCV format
            yolo_image = self.bridge.imgmsg_to_cv2(yolo_image_msg, 'bgr8')

            # Perform color segmentation (adjust these values based on your color properties)
            lower_color = np.array([0, 100, 100])
            upper_color = np.array([20, 255, 255])
            mask = cv2.inRange(yolo_image, lower_color, upper_color)
            segmented_image = cv2.bitwise_and(yolo_image, yolo_image, mask=mask)

            # Convert the segmented image back to ROS format and publish
            segmented_image_msg = self.bridge.cv2_to_imgmsg(segmented_image, 'bgr8')
            self.segmented_pub.publish(segmented_image_msg)

        except Exception as e:
            rospy.logerr(f"Error processing YOLO image: {str(e)}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        segmentation_node = ColorSegmentationNode()
        segmentation_node.run()
    except rospy.ROSInterruptException:
        pass
