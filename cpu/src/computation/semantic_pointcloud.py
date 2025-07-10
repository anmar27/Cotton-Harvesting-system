#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2

class SemanticPointCloudMerger:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('semantic_pointcloud_merger', anonymous=True)
        
        # Subscribers with time synchronization
        self.pc_sub = Subscriber("/camera/depth_registered/points", PointCloud2)
        self.semantic_sub = Subscriber("/rgb/segmented_image_raw", Image)
        
        # Publisher for semantic point cloud
        self.semantic_pc_pub = rospy.Publisher("/semantic_pointcloud", PointCloud2, queue_size=1)

        # Optional diagnostic publishers
        self.pc_debug_pub = rospy.Publisher("/debug/pointcloud_received", PointCloud2, queue_size=1)
        self.semantic_debug_pub = rospy.Publisher("/debug/semantic_image_received", Image, queue_size=1)
        # Time synchronizer
        self.ts = ApproximateTimeSynchronizer([self.pc_sub, self.semantic_sub], queue_size=10, slop=0.1) 
        self.ts.registerCallback(self.pointcloud_semantic_callback)
        
        # Semantic color mapping (adjust as needed)
        self.semantic_color_map = {
            (255, 0, 0): 0,   # Red class
            (0, 255, 0): 1    # Green class
        }

        # Diagnostic variables
        self.pc_count = 0
        self.semantic_count = 0
        # Setup diagnostic timer
        rospy.Timer(rospy.Duration(5), self.print_diagnostic_info)
    
    def print_diagnostic_info(self, event=None):
        """Print diagnostic information about received messages"""
        rospy.loginfo(f"Pointcloud messages received: {self.pc_count}")
        rospy.loginfo(f"Semantic image messages received: {self.semantic_count}")

    def pointcloud_semantic_callback(self, pointcloud_msg, semantic_img_msg):

        try:
            # Convert semantic image to OpenCV format
            semantic_img = self.convert_rosimg_to_cv2(semantic_img_msg)
            
            # Convert point cloud to numpy array
            pc_data = list(pc2.read_points(pointcloud_msg, field_names=["x", "y", "z", "rgb"], skip_nans=True))
            
            # Prepare new point cloud data with semantic information
            new_points = []
            
            # Iterate through point cloud
            for i, (x, y, z, rgb) in enumerate(pc_data):
                # Convert floating point rgb to int
                color_int = int(rgb)
                color_bgr = self.float_to_bgr(color_int)
                print("OKEY")
                # Get pixel coordinates (assuming point cloud and image have same dimensions)
                px = int(i % semantic_img.shape[1])
                py = int(i // semantic_img.shape[1])
                print("NOT OKEY")
                # Get semantic class from image
                try:
                    pixel_color = tuple(semantic_img[py, px])
                    semantic_class = self.semantic_color_map.get(tuple(pixel_color), -1)
                except IndexError:
                    semantic_class = -1
                
                # Create new point with semantic information
                new_point = [x, y, z, rgb, semantic_class]
                new_points.append(new_point)
            
            # Create new point cloud message
            new_fields = [
                pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                pc2.PointField('rgb', 12, pc2.PointField.FLOAT32, 1),
                pc2.PointField('semantic_class', 16, pc2.PointField.INT32, 1)
            ]
            
            semantic_pc_msg = pc2.create_cloud(pointcloud_msg.header, new_fields, new_points)
            
            # Publish semantic point cloud
            self.semantic_pc_pub.publish(semantic_pc_msg)
        
        except Exception as e:
            rospy.logerr(f"Error processing semantic point cloud: {e}")
    
    def convert_rosimg_to_cv2(self, ros_image):
        """Convert ROS image message to OpenCV image"""
        return np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
            ros_image.height, ros_image.width, -1
        )
    
    def float_to_bgr(self, float_rgb):
        """Convert float representation of RGB to BGR"""
        rgb_int = int(float_rgb)
        b = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        r = rgb_int & 0xFF
        return [b, g, r]

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        semantic_merger = SemanticPointCloudMerger()
        semantic_merger.run()
    except rospy.ROSInterruptException:
        pass



