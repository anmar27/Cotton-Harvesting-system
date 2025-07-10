#!/usr/bin/env python

import rospy
from computation.msg import BoundingBoxes  # Adjust the import if your message is different

def callback(data):
    # Print the received bounding boxes
    rospy.loginfo("Received bounding boxes:")
    for box in data.bounding_boxes:  # Assuming bounding_boxes is a list in your message
        rospy.loginfo(f"Class: {box.Class}, "
                      f"Confidence: {box.probability}, "
                      f"Xmin: {box.xmin}, "
                      f"Ymin: {box.ymin}, "
                      f"Xmax: {box.xmax}, "
                      f"Ymax: {box.ymax}")

def listener():
    rospy.init_node('bounding_boxes_listener', anonymous=True)  # Initialize the node
    rospy.Subscriber("/yolo/bounding_boxes", BoundingBoxes, callback)  # Subscribe to the topic
    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    listener()
