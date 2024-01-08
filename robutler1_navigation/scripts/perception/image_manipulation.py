#!/usr/bin/env python3

import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def parse_arguments():

    limits = {'color_lower':[0,0,0],'color_upper':[255,255,255]}
    parser = argparse.ArgumentParser(description="Color Segmentation ROS Node")
    parser.add_argument("-c","--color", type=str, default="all", help="Color to look for", required=False)
    args = parser.parse_args()

    if args.color == "all":
        limits['color_lower']=[0,0,0]
        limits['color_upper']=[255,255,255]
    
    elif args.color == "red":
        limits['color_lower'] = [0, 100, 100]  
        limits['color_upper'] = [10, 255, 255] 
    
    elif args.color == "blue":
        limits['color_lower'] = [90, 50, 50] 
        limits['color_upper'] = [110, 255, 255]

    else:
        print(f"Invalid color specified: {args.color}. Using default color range.")


    return limits

class ColorSegmentationNode:
    def __init__(self, color_lower, color_upper):
        rospy.init_node('image_manipulation_node', anonymous=False)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to the camera feed
        rospy.Subscriber('/yolov7/yolov7/visualization', Image, self.camera_callback)

        # Publisher for downsampled image
        self.downsampled_pub = rospy.Publisher('/manipulated_image', Image, queue_size=1)

        self.lower_color = np.array(color_lower)
        self.upper_color = np.array(color_upper)

    def camera_callback(self, camera_image_msg):
        try:
            # Convert camera image message to OpenCV format
            camera_image = self.bridge.imgmsg_to_cv2(camera_image_msg, 'bgr8')
            hsv_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2HSV)            

            mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)
            segmented_image = cv2.bitwise_and(camera_image, camera_image, mask=mask)

            # Downsample the image (adjust the scale factor based on your requirements)
            scale_factor = 0.5
            downsampled_image = cv2.resize(segmented_image, None, fx=scale_factor, fy=scale_factor)

            # Convert the downsampled image back to ROS format and publish
            downsampled_image_msg = self.bridge.cv2_to_imgmsg(downsampled_image, 'bgr8')
            self.downsampled_pub.publish(downsampled_image_msg)

        except Exception as e:
            rospy.logerr(f"Error processing camera image: {str(e)}")

    def run(self):
        rospy.spin()

# -------------------------------
# MAIN
# -------------------------------

if __name__ == '__main__':
    limits = parse_arguments()
    segmentation_node = ColorSegmentationNode(limits['color_lower'], limits['color_upper'])
    try:
        segmentation_node.run()
    except rospy.ROSInterruptException:
        pass
