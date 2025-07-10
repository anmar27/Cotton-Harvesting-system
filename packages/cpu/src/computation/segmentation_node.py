#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImage
from computation.msg import BoundingBoxes, BoundingBox, MaskInfo, MaskInfoArray
from cv_bridge import CvBridge, CvBridgeError
from utils.vision import Segmentation  
import numpy as np

CLASS_MAPPING = {
    0: "Cotton Unripe",
    1: "Cotton Ripe"
}

class SegmentationNode:
    def __init__(self):
        # ROS Node initialization
        rospy.init_node("segmentation_node", anonymous=True)

        # Subscribers and Publishers
        self.bounding_boxes_sub_ = rospy.Subscriber("/yolo/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
        self.masks_pub_ = rospy.Publisher("/segmentation_node/masks_info", MaskInfoArray, queue_size=1)

        # Initialize segmentation helper
        self.segmenter = Segmentation()
        self.bridge_ = CvBridge()

        rospy.loginfo("Segmentation node initialized.")

    def bounding_boxes_callback(self, msg):
        """
        Callback to process bounding boxes and perform segmentation.
        """

        current_time = rospy.Time.now()
        received_time = msg.header.stamp

        if((current_time.secs + current_time.nsecs * 1e-9)-(received_time.secs + received_time.nsecs * 1e-9)) <= 0.1:
            bounding_boxes = msg.bounding_boxes
        else: 
            bounding_boxes = []

        if bounding_boxes == []:
            #rospy.logwarn("No bounding boxes received.")
            return

        try:
            # Retrieve the latest image
            image_msg = rospy.wait_for_message("/camera/color/image_rect_color", ROSImage, timeout=0.1)
        except rospy.ROSException as e:
            rospy.logerr(f"Error retrieving image: {e}")
            return

        # Execute MobileSAM segmentation
        results = self.segmenter.executeMobileSAMSegmentation(image_msg, bounding_boxes)

        if results is None:
            rospy.logerr("Segmentation failed.")
            return

        print(results)

        # Create a MaskInfoArray message
        mask_info_array = MaskInfoArray()
        mask_info_array.header = msg.header  # Use the header from the original message

        for result in results:
            masks = result["masks"]  # List of masks
            centroids = result["centroids"]  # List of centroids
            bounding_boxes = result["bounding_boxes"]  # List of bounding boxes

            for i, mask in enumerate(masks):
                # Extract information for this mask
                centroid = centroids[i]
                box = bounding_boxes[i]

                # Get class ID and class name
                class_id = int(box.Class)
                class_name = CLASS_MAPPING.get(class_id, "unknown")

                # Create a MaskInfo message
                mask_info = MaskInfo()
                mask_info.header = msg.header
                mask_info.class_id = class_id
                mask_info.class_name = class_name
                mask_info.centroid_x = centroid[0]
                mask_info.centroid_y = centroid[1]

                
                mask_info_array.masks_info.append(mask_info)

        self.masks_pub_.publish(mask_info_array)
        rospy.loginfo(f"Published MaskInfoArray with {len(mask_info_array.masks_info)} detections.")

    def run(self):
        """Run the ROS node."""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = SegmentationNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Segmentation node terminated.")
