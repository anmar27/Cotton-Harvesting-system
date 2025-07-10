# Libraries
import pdb
import time
import math
import rospy
import numpy as np
import torch
from cv_bridge import CvBridge, CvBridgeError
from torchvision import transforms
import cv2
from utils.utils_PerceptionModel import utilPerceptionModel
from utils.utils_cv import rosImage2cv2image, cv2image2rosImage
from utils.utils_tf import generalUtilsTF
import threading

#YOLO
from ultralytics import YOLO

#K-nearest neighbors
from spicy import spatial

## Models
from ultralytics import YOLO
from mobile_sam import sam_model_registry, SamPredictor

# ROS
import tf
from sensor_msgs.msg import CameraInfo

## Utils
import tf.transformations
from utils.utils_geom import *

#TF
import tf
from geometry_msgs.msg import TransformStamped

# Types of return of Azure Kinect ROS node (rgb/image_raw, depth_to_rgb/image_raw)
from sensor_msgs.msg import (
    CameraInfo as ROSCameraInfo, 
    Image as ROSImage,
    PointCloud2,
)
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Time 
from std_msgs.msg import Header
# .srv
from computation.srv import TFpublisher, TFpublisherResponse, getDetectedInfo

from computation.msg import BoundingBox


class Segmentation(object):
    """
    Segmentation class. Used for: 
    - Box segmentation with MobileSAM
    - Centroid computation 
    Additionally localization functionalities are added here.
    """
    #Private methods

    def __combineMaskImage(self, masks, bounding_boxes):
        """
        Combine masks and assign RGB colors based on class.

        Args:
            masks (list of np.ndarray): List of boolean mask arrays of shape (C, H, W).
            bounding_boxes (list): List of bounding box information with keys:
                                   'xmin', 'ymin', 'xmax', 'ymax', 'Class'.

        Returns:
            mask_image (np.ndarray): Color image with masks applied, shape (H, W, 3).
        """
        if not all(isinstance(mask, np.ndarray) for mask in masks):
            raise ValueError("All masks must be numpy arrays.")

        # Define RGB colors for classes
        class_colors = {
            0.0: (255, 0, 0),  # Red for class 0.0
            1.0: (0, 255, 0)   # Green for class 1.0
        }

        # Determine the size of the image from the first mask
        _, H, W = masks[0].shape

        # Create a blank canvas for the combined colored mask
        combined_mask = np.zeros((H, W, 3), dtype=np.uint8)

        # Iterate over bounding boxes and corresponding masks
        for mask, box in zip(masks, bounding_boxes):

            class_id = box.Class
            xmin, ymin, xmax, ymax = map(int, [box.xmin, box.ymin, box.xmax, box.ymax])

            # Get corresponding mask slice and color
            color = class_colors.get(class_id, (255, 255, 255))  # Default to white if class is unknown

            # Extract the mask area within the bounding box
            mask_slice = mask[:, ymin:ymax, xmin:xmax]

            # Apply the mask to the canvas with the given color
            for c in range(3):  # Apply RGB color
                combined_mask[ymin:ymax, xmin:xmax, c] = np.where(mask_slice.any(axis=0), color[c], combined_mask[ymin:ymax, xmin:xmax, c])

        # Convert to ROS image format
        mask_image = cv2image2rosImage(combined_mask, "rgb8")

        # Add a header to the message
        mask_image.header = Header()
        mask_image.header.stamp = rospy.Time.now()  # Current timestamp
        mask_image.header.frame_id = "camera_link"
        return mask_image

    def __setMaskValues(self,centroid):
        self.centroidValues = centroid

    #TODO -> Likely deprecated
    def __segment(self, image, box: np.ndarray) -> np.ndarray:
        """
        Returns the list of pixels belonging to the actual detected object.
        Receives a subregion of the image, given by box = (x_min, y_min, x_max, y_max) in YOLO format.
        MobileSAM accepts points in image as prompts too.
        TODO maybe change this to only segment a portion of the image?
        """
        self.sam.set_image(image)
        mask, _, _ = self.sam.predict(box=box)
        return mask    # TODO Warning! mask = (CxHxW), where C is the number of mask --> C has to be 1!

    def __loadModels(self):
        sam_checkpoint = rospy.get_param('/detection_parameters/mobile_sam_path')
        model_type = "vit_t"
        device = "cuda" if torch.cuda.is_available() else "cpu"
        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        sam.to(device=device)
        sam.eval()

        self.predictor = SamPredictor(sam)

        ## Centroid computation
    #TODO code currently not implemented for pseudo centroid -> Needed to test performance between both methods 
    def __compute_pseudocentroid(self, coords) -> tuple:
        """
        Computes the pseudo-centroid of a given subregion in image.

        We call pseudo-centroid the center of a predicted label, but does not necessarily translates 
        to the centroid of the actual object.

        coords = (xmin, ymin, xmax, ymax)
        """
        x = (coords[0] + coords[2]) // 2
        y = (coords[1] + coords[3]) // 2
        return (x,y)

    def __compute_centroid(self, mask: np.ndarray) -> tuple:
        """
        Computes the actual centroid of a region of image. The region is given by a mask, 
        concerning which pixels belong to the actual object.

        mask is of the form MxNx1 == MxN
        """

        if len(mask.shape) == 3 and mask.shape[0] != 1:
            mask = mask[0]  # Remove the first dimension
        else:
            rospy.logerr("Unspected error in the mask segmented.")
            raise ValueError("Unspected error in mask segmented! Likely to have passed mask with wrong dimensions")
    
        # Get the indices of the True values
        indices = np.argwhere(mask)
    
        if indices.size == 0:
            rospy.logerr("Unspected error in the mask segmented.")
            raise ValueError("Mask does not contain True values!")
    
        # Calculate the centroid as the median of the indices
        x_centroid = np.median(indices[:, 0]).item()
        y_centroid = np.median(indices[:, 1]).item()
        
        return [x_centroid,y_centroid]

    def __init__(self) -> None:
        
        self.segmentBridge_ = CvBridge() 

        self.__loadModels()

        #Initalize past variables
        self.centroidValues = None

        #Publishers
        self.segmented_mod_image_pub_ = rospy.Publisher('/rgb/segmented_image_raw', ROSImage, queue_size=50)

    #Public methods
    def executeMobileSAMSegmentation(self, image_raw, bounding_boxes):
        """
        Processes an image and a list of bounding boxes using MobileSAM to segment objects and compute centroids.
        Centroid is computed only for the first mask for each bounding box.
    
        Args:
            image_raw (ROSImage): The raw image message to process.
            bounding_boxes (list[BoundingBox]): A list of bounding boxes to segment.
    
        Returns:
            list[dict]: A list of dictionaries containing mask, centroids, and bounding box details.
        """
        #Initialze arrays
        results = []
        masks = []
        centroids = []

        # Transform ROS image to cv2 format
        try:
            cv_image = rosImage2cv2image(image_raw, "bgr8")
        except ValueError as err:
            rospy.logwarn(f"Invalid image format: {err}")
            return None
        except CvBridgeError as err:
            rospy.logwarn(f"CvBridge Error: {err}")
            return None
    
        # Set image for MobileSAM predictor
        self.predictor.set_image(cv_image)
    
        # Process each bounding box
        for box in bounding_boxes:
            # Create a list of box coordinates for the current bounding box
            box_coords = [[box.xmin, box.ymin, box.xmax, box.ymax]]

            # Convert to a NumPy array with dtype float32 and ensure it's a 2D array (shape: [1, 4])
            box_array = np.array(box_coords, dtype=np.float32)
            #print("Box Array Shape: ", box_array.shape)

            # Perform segmentation through MobileSAM
            mask, _, _ = self.predictor.predict(
                point_coords=None,
                point_labels=None,
                box=box_array,  # Pass a single bounding box (shape: [1, 4])
                multimask_output=True,  # To get all mask
            )

            if mask is None or len(mask) == 0:
                rospy.logwarn(f"No mask generated for bounding box: {box_coords}")
                continue  # Skip to the next bounding box if no mask is generated

            # Compute centroid for the first mask only
            try:
                centroid = self.__compute_centroid(mask)  # Compute centroid for the first mask
            except ValueError as err:
                rospy.logwarn(f"Error computing centroid: {err}")
                centroid = None  # Set centroid to None if computation fails

            # Publish combined mask image for the first mask

            
            #Populate arrays
            masks.append(mask)
            centroids.append(centroid)

        #combined_image = self.__combineMaskImage(masks,bounding_boxes)
        #self.segmented_mod_image_pub_.publish(combined_image)

        # Collect results for the current bounding box
        results.append({
            "masks": masks,  # All mask for this bounding box
            "centroids": centroids,  # Centroid for the first mask
            "bounding_boxes": bounding_boxes  # Original bounding box
        })

        #Set new centroid values for SM functioning
        self.__setMaskValues(centroids[0])

        return results

# Main Code
def main():
    # Initialize the segmentation instance
    segmentation = Segmentation()

    # Create an instance of CvBridge for conversions
    bridge = CvBridge()

    # Load the image directly from a file
    image_path = "/iri_lab/iri_ws/src/cpu/src/computation/first.png"  # Replace with your image path
    cv_image = cv2.imread(image_path)

    if cv_image is None:
        print(f"Failed to load image from {image_path}")
        return

    # Convert OpenCV image to ROS Image
    try:
        image_raw = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")  # Convert to ROSImage
    except Exception as e:
        print(f"Failed to convert OpenCV image to ROSImage: {e}")
        return

    # Define a single bounding box
    bounding_box = BoundingBox(xmin=620, ymin=124, xmax=889, ymax=337)
    bounding_boxes = [bounding_box]

    # Call the segmentation method
    results = segmentation.executeMobileSAMSegmentation(image_raw, bounding_boxes)

    print("RESULTS")
    print(results)

    # Handle results
    if results and results[0]["masks"]:
        mask = results[0]["masks"][0]
        mask = np.clip(mask, 0, 1)
        mask = mask[0]
        print("MASK")
        print(mask)                 # Get the binary mask for the first bounding box
        print(f"Mask shape: {mask.shape}")
        centroid = results[0]["centroids"][0]

        # Print centroid
        print(f"Centroid: {centroid}")

        # Save the binary mask
        mask_output_path = "/iri_lab/iri_ws/src/cpu/src/computation/frame_0.png"
        success = cv2.imwrite(mask_output_path, (mask * 255).astype(np.uint8))
        print(f"Binary mask saved at {mask_output_path}")
        if not success:
            print("Failed to save binary mask.")
    else:
        print("Segmentation failed or no masks found.")

if __name__ == "__main__":
    main()