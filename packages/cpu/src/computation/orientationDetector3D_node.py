#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import message_filters
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf.transformations
import struct

class PointCloudOrientationDetector:
    def __init__(self):
        rospy.init_node('pointcloud_orientation_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribers
        self.seg_sub = message_filters.Subscriber('/rgb/segmented_image_raw', Image)
        self.cloud_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)
        
        # Time synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.seg_sub, self.cloud_sub], 
            queue_size=10, 
            slop=0.1
        )
        ts.registerCallback(self.callback)
        
        # Publishers
        self.marker_pub = rospy.Publisher('/object_orientation_markers', 
                                        MarkerArray, queue_size=1)     

    def point_cloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 to numpy array while preserving organization"""
        # Get cloud dimensions
        height = cloud_msg.height
        width = cloud_msg.width
        
        # Initialize arrays for x, y, z coordinates
        points = np.full((height, width, 3), np.nan)
        
        # Get the positions of x, y, z in the point cloud fields
        fields_dict = {f.name: i for i, f in enumerate(cloud_msg.fields)}
        
        # Convert point cloud data to array
        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step
        
        # Extract raw data
        cloud_data = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(height, width, -1)
        
        # Extract x, y, z values
        for i in range(height):
            for j in range(width):
                x = struct.unpack('f', cloud_data[i, j, 0:4])[0]
                y = struct.unpack('f', cloud_data[i, j, 4:8])[0]
                z = struct.unpack('f', cloud_data[i, j, 8:12])[0]
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    points[i, j] = [x, y, z]
        
        return points

    def get_3d_orientation(self, points):
        """Calculate 3D orientation using PCA on point cloud data"""

        valid_points = points[~np.isnan(points).any(axis=1)]
    
        if len(valid_points) < 3: #At least three points required
            return None, None, None

        print(len(valid_points))
        #centroid = np.mean(valid_points, axis=0) #mean
        centroid = np.median(valid_points, axis=0)
        centered_points = valid_points - centroid
        
        # Compute covariance matrix and its eigenvectors
        covariance_matrix = np.cov(centered_points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        
        #Sorting eingenvectors by eigenvalues in descending order
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        print("EUIGEN")
        print(eigenvalues)
        print("VECTORS")
        print(eigenvectors)
        eigenvectors = eigenvectors[:, idx]
        
        #First eigenvector represents primary axis
        primary_axis = eigenvectors[:, 0]
        secondary_axis = eigenvectors[:, 1]
        normal_vector = eigenvectors[:, 2]  # Smallest eigenvector
        
        return (eigenvectors, eigenvalues), centroid, valid_points


    def create_orientation_marker(self, position, vectors, eigenvalues, instance_id, scale_factor=500):
        """Create markers showing the object's principal axes"""
        colors = [(1,0,0), (0,1,0), (0,0,1)]  # RGB colors for each axis
        markers = []
        
        for i, (vector, color) in enumerate(zip(vectors.T, colors)):
            marker = Marker()
            marker.header.frame_id = "camera_depth_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"cotton_boll_{instance_id}"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(3)
            
            #Use the centroid as origin of markers
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            
            # Calculate orientation for this axis
            axis_matrix = np.eye(3)
            axis_matrix[:, 0] = vector
            # Complete orthonormal basis
            axis_matrix[:, 1] = np.cross(vector, np.array([0, 0, 1]))
            axis_matrix[:, 1] /= np.linalg.norm(axis_matrix[:, 1])
            axis_matrix[:, 2] = np.cross(axis_matrix[:, 0], axis_matrix[:, 1])
            
            quaternion = tf.transformations.quaternion_from_matrix(
                np.vstack([np.hstack([axis_matrix, position.reshape(-1, 1)]),
                          [0, 0, 0, 1]])
            )
            
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            
            #Scale length based on eigenvalues
            marker.scale.x = scale_factor*eigenvalues[i]  # Length
            marker.scale.y = 0.01  # Width
            marker.scale.z = 0.01  # Height
            
            # Set the color
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            markers.append(marker)
        
        return markers

    def callback(self, seg_msg, cloud_msg):
        
        
        cv_image = self.bridge.imgmsg_to_cv2(seg_msg, "rgb8")
        point_array = self.point_cloud2_to_array(cloud_msg)
        
        #Find instances of detections
        green_mask = cv_image[:, :, 1] == 255  # Green channel
        white_mask = (cv_image[:, :, 0] == 255) & (cv_image[:, :, 1] == 255) & (cv_image[:, :, 2] == 255)  # White mask
        blue_mask = (cv_image[:, :, 0] == 0) & (cv_image[:, :, 1] == 0) & (cv_image[:, :, 2] == 255)  # Pure blue mask
        
        binary = white_mask | green_mask #Adjust depending of objec of interested (class information encoded on code)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary.astype(np.uint8), connectivity=8)
        min_size = 150 #Num of pixels in case of discarding

        filtered_mask = np.zeros_like(binary)

        #Iterate over components (excluding background label 0)
        for i in range(1, num_labels):  
            if stats[i, cv2.CC_STAT_AREA] >= min_size:  
                filtered_mask[labels == i] = 255


        print("LABELS")
        print(num_labels)
        print(labels)
        # Create marker array
        marker_array = MarkerArray()
        
        # Process each instance
        for instance_id in range(1, num_labels):
            #Mask per instance
            instance_mask = (labels == instance_id)
            
            #3D points from instance mask
            instance_points = point_array[instance_mask]
            
            #Calculate 3D orientation
            orientation_data = self.get_3d_orientation(instance_points)
            #print("Orientation data")
            #print(orientation_data)
            if orientation_data[0] is not None:
                (vectors, eigenvalues), centroid, valid_points = orientation_data
                #print("Vectors")
                #print(vectors)
                #print(eigenvalues)
                # Create markers for each principal axis
                instance_markers = self.create_orientation_marker(
                    centroid, vectors, eigenvalues, instance_id)
                marker_array.markers.extend(instance_markers)
                #print(marker_array)
        # Publish markers
        self.marker_pub.publish(marker_array)
            

    def run(self):
        #Keep run running
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = PointCloudOrientationDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass