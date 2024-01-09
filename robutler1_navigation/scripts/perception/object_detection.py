#!/usr/bin/env python3

import rospy
import argparse
from std_msgs.msg import String
from std_msgs.msg import Bool

def callback(data, target_object):
    rospy.loginfo("Received message: %s", data.data)
    
    # Check if the target string is the received data string
    if target_object.lower() in data.data.lower():
        result_msg = Bool(data=True)
        result_pub.publish(result_msg)
        rospy.loginfo("Detected: True")
    else:
        result_msg = Bool(data=False)
        result_pub.publish(result_msg)
        rospy.loginfo("Detected: False")

def listener(object):
    rospy.init_node('object_identification', anonymous=False)
    
    # Subscribe to the 'string_topic' topic
    rospy.Subscriber('/yolov7/detected_objects', String, callback, callback_args=object)
    
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ROS object identification Listener')
    parser.add_argument('-o','--object', type=str, help='Target string to compare with (e.g., "sports ball")')
    args = parser.parse_args()

    # Create a publisher for the result message on the 'result_topic' topic
    result_pub = rospy.Publisher('result_topic', Bool, queue_size=10)
    
    try:
        listener(args.object)
    except rospy.ROSInterruptException:
        pass
