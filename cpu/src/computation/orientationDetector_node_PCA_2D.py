#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OrientationDetector:
    def __init__(self):
        rospy.init_node('orientation_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribe to the segmented image topic
        self.image_sub = rospy.Subscriber('/rgb/segmented_image_raw', Image, 
                                        self.image_callback)
        
        # Publisher for the visualization
        self.viz_pub = rospy.Publisher('/object_orientation_viz', Image, 
                                     queue_size=1)
        
    def get_orientation_pca(self, binary_mask):
        """Calculate orientation using PCA for a single instance"""
        # Get coordinates of non-zero points
        coords = np.column_stack(np.where(binary_mask > 0))
        
        if len(coords) < 2:  # Need at least 2 points for PCA
            return None, None, None
        
        # Perform PCA
        mean = np.mean(coords, axis=0)
        coords_centered = coords - mean
        covariance_matrix = np.cov(coords_centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        
        # The largest eigenvector gives the main axis orientation
        angle = np.arctan2(eigenvectors[-1, 1], eigenvectors[-1, 0])
        return np.degrees(angle), mean, eigenvectors[-1]
    
    def find_instances(self, mask):
        """Find and separate different instances in the mask"""
        # Threshold the green channel (assuming 255 in green channel marks objects)
        binary = mask[:, :, 1] == 255
        
        # Find connected components
        num_labels, labels = cv2.connectedComponents(binary.astype(np.uint8))
        
        return labels, num_labels
    
    def draw_orientation(self, image, center, direction, angle, color=(0, 0, 255), 
                        line_length=50):
        """Draw an arrow indicating the orientation"""
        start_point = tuple(map(int, center))
        end_point = tuple(map(int, [
            center[0] + line_length * direction[0],
            center[1] + line_length * direction[1]
        ]))
        
        cv2.arrowedLine(image, start_point, end_point, color, 2)
        
        # Add angle text
        cv2.putText(image, f"{angle:.1f}°", 
                    (start_point[0] + 10, start_point[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # Find separate instances
            labels, num_instances = self.find_instances(cv_image)
            
            # Create visualization image (copy of original)
            viz_image = cv_image.copy()
            
            # Process each instance
            for instance_id in range(1, num_instances):  # Skip 0 (background)
                # Create mask for this instance
                instance_mask = (labels == instance_id)
                
                # Calculate orientation
                angle, center, direction = self.get_orientation_pca(instance_mask)
                
                if angle is not None:
                    # Make angle positive (0-180 degrees)
                    if angle < 0:
                        angle += 180
                    
                    # Draw orientation arrow
                    self.draw_orientation(viz_image, center, direction, angle)
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, "rgb8")
            self.viz_pub.publish(viz_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = OrientationDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass