"""
Computer vision module to compute all things related to detection, classification and localization:

- Cotton detection and classification (YOLO)
- Semantic segmentation (YOLO)
- Centroid computation (depth obtaining and calculation)

TODO things:

 - Pasar todas las class variables a un YAML/XML/whatever para guardar y manipular los atributos de manera más eficiente
    (y cargar y guardar esos datos cada vez que sea necesario) --> TODO HACER
 - Cómo nos aseguramos que la iésima predicción en una foto se corresponde con la iésima máscara de SAM?

 - Las funciones que no son propias de la visión (computación de centroide, profundidad...) debería moverlas a otro módulo utils?
"""
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
# .srv
from computation.srv import TFpublisher, TFpublisherResponse, getDetectedInfo

from computation.msg import MaskInfo, MaskInfoArray


#Auxiliar Classes
class rosImage2:
    """
    Class in charge of creating the conversion to other formats, returning cv2 image
    """
    def __init__(self,ros_image: ROSImage):

        self.bridge_ = CvBridge()

        try:
            cv_image = self.bridge_.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError as err:
                print(err)    
        self.cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)


    def getCv2Image(self):
        return self.cv_image

class rosImage2Tensor(rosImage2):
    """
    Sub-Class from RosImage2 which return the image in tensor format
    """
    def __init__(self, ros_image: ROSImage):
        super().__init__(ros_image)
        self.tensor_image = None

        if self.cv_image is not None:
            # Convert cv_image (OpenCV) to tensor
            transform = transforms.ToTensor()
            self.tensor_image = transform(self.cv_image)

            # Convert BGRA to BGR (if needed)
            if self.cv_image.shape[2] == 4:  # If it has 4 channels (BGRA)
                self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGRA2BGR)

            # If the image has 3 channels (BGR), convert it to RGB in the tensor
            if self.tensor_image.shape[0] == 3:
                self.tensor_image = self.tensor_image[[2, 1, 0], :, :]  # Swap B and R channels (BGR to RGB)
        else:
            rospy.loginfo("Error in the pre-conversion to cv image")
            
    
    def getTensorImage(self):
        return self.tensor_image

class Detection(object):
    """
    Computer vision class. Used for: 
    - Detecting the cotton bolls with YOLO model
    """

    # Private Methods

    def __loadModels(self,
                     yolo_path: str) -> None:
        """
        Private method. Loads the YOLOv8 and SAM model into object.
        """
        torch.cuda.set_device(0)
        self.yolo_model = YOLO(yolo_path)      
    
    #TODO duplication of methods in Vision and utils_PerceptionModel (necessary to refactor code)
    def __setDetectedInfo(self,boxes,classes,confidences):
        self.boxesInfo_ = boxes
        self.classesInfo_ = classes
        self.confidencesInfo_ = confidences
    
    def __checkReadyToHarverst(self,boxes,classes,confidences):
        #TODO change to ros_parameter
        confidenceThreashold = rospy.get_param("/detection_parameters/confidence_threshold")
        for confidence in confidences:
            if confidence > confidenceThreashold:
                return True
        return False
    
    def __get_approximate_centroids(self,boxes: torch.Tensor):
        centroids = []
        #Box -> [center_x,center_y,w,h]
        for box in boxes:
            x_centerPoint = box[0]
            y_centerPoint = box[1]
            center = [x_centerPoint.item(),y_centerPoint.item()]
            centroids.append(center)
        return centroids

    def __setInfoDetections(self,boxes,classes,confidences):
        self.boxesInfo_ = boxes
        self.classesInfo_ = classes
        self.confidencesInfo_ = confidences
    
    def getInfoDetections(self):            
        return self.boxesInfo_,self.classesInfo_,self.confidencesInfo_

    def __obtainStatesAndApproxCentroids(self,cv_image): #Get rid of this in future
        start_time = time.time()
        #Inferance of Yolov8 model
        #print("Entering YOLO Modified Image step")
        results = self.yolo_model(cv_image)
        boxes = results[0].boxes.xywh
        boxes_xyxy = results[0].boxes.xyxy.tolist()
        classes = results[0].boxes.cls.tolist()
        confidences = results[0].boxes.conf.tolist()
        names = results[0].names
        if boxes == []:
            YOLO_computation_time = time.time() - start_time
            return boxes, YOLO_computation_time,cv_image
        else:
            #print("Boxes obtained" + str(boxes))
            aprox_centroids = self.__get_approximate_centroids(boxes)
            for box,cls,conf in zip(boxes_xyxy, classes,confidences):
                    #TODO add index number of each localization in mod_image
                    x_min, y_min, x_max, y_max = box
                    cls = cls
                    conf = conf
                    cv2.rectangle(cv_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                    # Create text with class and confidence
                    text = f"{names[int(cls)]}: {conf:.2f}" 
                    text_width, text_height = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)[0]
                    text_x = x_min + 5 
                    text_y = y_min - text_height  
                    # Draw the text with a black background for better visibility
                    cv2.rectangle(cv_image, (int(text_x), int(text_y - 5)), (int(text_x + text_width + 10), int(text_y + 5)), (0, 255, 0), cv2.FILLED) 
                    cv2.putText(cv_image, text, (int(text_x), int(text_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)  

            YOLO_computation_time = time.time() - start_time
            aprox_centroids = [[int(elem) for elem in sublist] for sublist in aprox_centroids]
            self.__setInfoDetections(boxes_xyxy,classes,confidences)
            return aprox_centroids, YOLO_computation_time,cv_image
    ## Callback from stream ROSImage
    def get_predictions(self, image) -> tuple: # INPUT: rgb/image_raw (5Hz)
        """
        Predicts a single image once a image appears in the rgb/image_raw pipeline.

        #TODO Returns the predicted labels, scores and clases, in the form of a list of dictionaries.
        """
        self.cv_image = rosImage2(image)
        cv_image = self.cv_image.getCv2Image()

        #Call YOLO for states and aprox centers
        aprox_centroids, execution_time, cv_image_points = self.__obtainStatesAndApproxCentroids(cv_image)
        
        #TODO get rid of reception_flag -> use instead size of aprox_centroids 

        if aprox_centroids == []:
           #rospy.loginfo(rospy.get_caller_id() + " Did not found matches in this frame")
           self.aproxCentroidsImage_ = self.bridge_.cv2_to_imgmsg(cv_image_points, "bgr8")
           #rospy.loginfo(rospy.get_caller_id() + " Model execution time: %s", str(execution_time))
           predictions = ("This", "is", "atest#1")
           return False

        else:
            rospy.loginfo(rospy.get_caller_id() + " Found matches in this frame")
            if rospy.get_param('/detection_parameters/camera_used') == 'kinova':
                self.aproxCentroidsImage_ = self.bridge_.cv2_to_imgmsg(cv_image_points, "rgb8") #Kinova
            else:
                self.aproxCentroidsImage_ = self.bridge_.cv2_to_imgmsg(cv_image_points, "bgr8") #Azure
            #rospy.loginfo(rospy.get_caller_id() + " Model execution time: %s", str(execution_time))
            #rospy.loginfo(rospy.get_caller_id() + " Model Aprox points founded: %s", str(aprox_centroids))  
            predictions = ("This", "is", "atest#2")
            
            #Publishing modified image
            self.detected_mod_image_pub_.publish(self.aproxCentroidsImage_)

            if self.__checkReadyToHarverst(*self.getInfoDetections()) == True:
                self.__setDetectedInfo(*self.getInfoDetections())
            
            return True
            
    ## Constructor
    def __init__(self) -> None:
            
        """Constructor of class."""        
        self.__loadModels(rospy.get_param("/detection_parameters/yolo_path")) #rosparam 

        #Initialize to None Detection Information variables
        self.boxesInfo_ = None
        self.classesInfo_ = None
        self.confidencesInfo_ = None 
        
        #Creation of Publishers 
        self.detected_mod_image_pub_ = rospy.Publisher('rgb/mod_image_raw', ROSImage, queue_size=50)
        self.depth_image_pub_ = rospy.Publisher('depth/cv_image', ROSImage, queue_size=50)

        #Creating CVBridge object for conversion from ROSImage to CV2
        self.bridge_ = CvBridge()
        
    def getDetectedInfo(self):
        return self.boxesInfo_, self.classesInfo_, self.confidencesInfo_

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
        self.segmented_mod_image_pub_ = rospy.Publisher('rgb/segmented_image_raw', ROSImage, queue_size=50)

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

        print("FFFF")
        print(masks)
        print(bounding_boxes)

        combined_image = self.__combineMaskImage(masks,bounding_boxes)
        self.segmented_mod_image_pub_.publish(combined_image)

        # Collect results for the current bounding box
        results.append({
            "masks": masks,  # All mask for this bounding box
            "centroids": centroids,  # Centroid for the first mask
            "bounding_boxes": bounding_boxes  # Original bounding box
        })

        #Set new centroid values for SM functioning
        self.__setMaskValues(centroids[0])

        return results

    """
        print(response_info.bounding_box.bounding_boxes[0])
        image_raw = response_info.image

        #Transform into cv2Image
        #TODO Change image_format into ros_param or smach passed value
        try:
            cv_image = rosImage2cv2image(image_raw,"bgr8")
        except ValueError as err:
            rospy.logwarn("Invalid image format: {}".format(err))
            return False
        except CvBridgeError as err:
            rospy.logwarn("CvBridge Error: {}".format(err))
            return False

        self.predictor.set_image(cv_image)
        
        #TODO by the moment just take the first one, needed to introduce selection mechanism 
        boxInfo = response_info.bounding_box.bounding_boxes[0]
        boxCoordinates = [boxInfo.xmin,boxInfo.ymin,boxInfo.xmax,boxInfo.ymax]
        boxInfo = np.array(boxCoordinates)

        mask, _, _ = self.predictor.predict(
            point_coords=None,
            point_labels=None,
            box=boxInfo[None, :],
            multimask_output=False,
        )

        try:
            centroidMask = self.__compute_centroid(mask)
        except ValueError as err:
            return False
            
        #Set values
        self.__setMaskValues(centroidMask)

        #Publish modified image with the mask
        self.combined_image = self.__combineMaskImage(mask)
        self.segmented_mod_image_pub_.publish(self.combined_image)

        return True
    """
    #TODO -> Temporal function to keep the Sm working -> NOW NEEDED TO RUN SEGMENTATION_NODE
    def serviceRequestSegmentation(self,detectionPassedInfo) -> bool:        

        #Infinite loop until info is retrieved
        while True:
            #Retrieve detected info from capture node
            detectedInfo_client = rospy.ServiceProxy('/capture_node/get_info', getDetectedInfo)

            
            timeObj = Time()
            timeObj.data = rospy.Time.now()
            response_info = detectedInfo_client(timeObj)
            if response_info.bounding_box.bounding_boxes == []:
                rospy.loginfo("Retrieved info from capture node empty")
                print(response_info.bounding_box)
                rospy.sleep(0.5)
            else:
                rospy.loginfo("Retrieved info from capture node not empty")
                print(response_info.bounding_box)
                break   

        #Declare pointcloud
        self.point_cloud = response_info.pointcloud
        
        
        return True

    def getCentroidValue2DSpace(self):
        return self.centroidValues

    def getPointCloud(self):
        return self.point_cloud

class DepthLocalization(object):

    def __init__(self, centroid2D,point_cloud) -> None:

        #TODO -> Add mechanism that double checks imput resolution from camera

        #Values kinova vision rgb camera
        resolution = rospy.get_param("/detection_parameters/resolution_camera")
        self.height = resolution['height']
        self.width = resolution['width']

        self.subsetSize = rospy.get_param("/detection_parameters/subset_knn") # Size of the grid in case of NaN value
        self.kNeighbours = rospy.get_param("/detection_parameters/knn_neighbours")

        #self.__centroid2D = centroid2D
        self.tf_cotton_array_ = None
        self.bridge_ = CvBridge()

        self.utils_tf = generalUtilsTF()
        
        self.point_cloud = point_cloud
        self.__centroid2D = None  # to store (y, x)
        self._masks_info_sub = rospy.Subscriber("/segmentation_node/masks_info", MaskInfoArray, self._callback_masksInfo)
        
    def _callback_masksInfo(self, msg):
        if not msg.masks_info:
            rospy.logwarn("Received empty masks_info array.")
            return

        #pick the first object of interest
        mask = msg.masks_info[0]

        # You can apply filtering logic here if needed
        self.__centroid2D = (mask.centroid_y, mask.centroid_x)
        rospy.loginfo(f"Updated centroid from mask: ({mask.centroid_y}, {mask.centroid_x})")
    
    def _extractSubset(self,point_list,target_index, subset_size=5):
        """
        Extracts a subset of points in a grid from the point cloud point list.
        """
        image_width = self.width
        step = int((subset_size -1)/2)
        init = target_index - step - step*image_width
        subset = []

        for i in range(subset_size):
            row_start = init + i * image_width   
            row_end = row_start + subset_size  
            subset.extend(point_list[row_start:row_end])

        return subset

    def _knnWhenNaN(self,subset,k):
        positionMidPixel = math.floor(self.subsetSize/2)
        subsetGrid = np.array(subset).reshape(self.subsetSize, self.subsetSize, 3)
        middle_idx = [positionMidPixel,positionMidPixel]

        # Extract non-NaN points for comparison, and ignore NaN values in x, y, or z
        valid_points = []
        for i in range(self.subsetSize):
            for j in range(self.subsetSize):
                point = subsetGrid[i, j]
                if not np.isnan(point).any():  
                    valid_points.append(((i, j), point))

        # If no valid points are found, return None
        if len(valid_points) == 0:
            return None

        print("VALID POINTS")
        print(valid_points)
        positions = np.array([p[0] for p in valid_points])  
        values = np.array([p[1] for p in valid_points])  

        tree = spatial.KDTree(positions)

        # Find the k-nearest neighbors to the middle pixel
        distances, indices = tree.query([middle_idx], k=min(k, len(valid_points)))

        print("Distances")
        print(distances)
        print(indices)

        # Get the corresponding (x, y, z) values of the k-nearest neighbors
        nearest_neighbors = values[indices[0]]

        # Compute the average of the nearest neighbors
        avg_xyz = np.mean(nearest_neighbors, axis=0)

        return avg_xyz
    
    def _obtainTargetIndex(self,targetRow, targetCol):
        return targetRow * self.width + targetCol

    #TODO -> Likely deprecated no longer used
    def _callback_depthImage(self,data):
        depth_image_cv = self.bridge_.imgmsg_to_cv2(data,"passthrough")  

        centroid = self.__centroid2D
        x,y = centroid
        depth_value_centroid = depth_image_cv[int(x),int(y)] 
        print("Centroid Distance: "+ str(depth_value_centroid) + " mm")
        print("Pixel: " + str(centroid))

    def _isWithinBounds(self, row, col):
        return row < self.height and col < self.width

    def _isValidIndex(self, index, length):
        return 0 <= index < length

    def _isAnyNan(self, x, y, z):
        return math.isnan(x) or math.isnan(y) or math.isnan(z)

    def _handleNanPoints(self, point_list, target_index):
        subset = self._extractSubset(point_list, target_index, self.subsetSize)
        newXYZ = self._knnWhenNaN(subset, k=self.kNeighbours)

        if newXYZ is None:
            rospy.logerr(f"Cannot obtain approximate centroid, object of interest likely too close")
            return np.zeros(3)

        return newXYZ

    def _logInfoPoint(self, idx, row, col, x, y, z):
        if x is None or y is None or z is None:
            rospy.logwarn(f"No valid point data at ({row}, {col})")
        else:
            rospy.loginfo(f"Point at ({row}, {col}): x={x}, y={y}, z={z} in meters")

    #TODO-> move functionality tf_utils
    def _createTransform(self,idx,x,y,z):

        child_string = f"cotton_frame_{idx}"
        t = TransformStamped()
        t.header.stamp = rospy.Time().now()

        if rospy.get_param('/detection_parameters/camera_used') == 'kinova':
            t.header.frame_id = 'camera_color_frame' 
        else:
            t.header.frame_id = 'rgb_camera_link'

        t.child_frame_id = child_string
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        #Rotation z axis parallel ground
        if rospy.get_param('/detection_parameters/camera_used') == 'kinova':
            newBaselink_H_camera = self.utils_tf.get_desired_tf("new_base_link","camera_color_frame")
        else:
            newBaselink_H_camera = self.utils_tf.get_desired_tf("new_base_link","rgb_camera_link")

        newBaselink_quaternion_camera = [newBaselink_H_camera.transform.rotation.x, newBaselink_H_camera.transform.rotation.y, newBaselink_H_camera.transform.rotation.z, newBaselink_H_camera.transform.rotation.w ]

        t.transform.rotation.x = newBaselink_quaternion_camera[0]
        t.transform.rotation.y = newBaselink_quaternion_camera[1]
        t.transform.rotation.z = newBaselink_quaternion_camera[2]
        t.transform.rotation.w = newBaselink_quaternion_camera[3]

        return t

    def callback_pointCloud(self):
        idx = 0
        array_transforms = []

        while True:
            if self.__centroid2D is None:
                rospy.logwarn("Centroid not yet received from /segmentation_node/masks_info")
                self.tf_cotton_array_ = array_transforms
            else:
                break
            
        rospy.loginfo("CENTROIDDD")
        rospy.loginfo(self.__centroid2D[1])
        rospy.loginfo(self.__centroid2D[0])
        
        
        target_row = int(self.__centroid2D[1]) 
        target_col = int(self.__centroid2D[0])

        point_list = list(pc2.read_points(self.point_cloud, field_names=("x", "y", "z")))


        if not self._isWithinBounds(target_row, target_col):
            rospy.logwarn(f"Target row {target_row} or column {target_col} is out of bounds")
            self.tf_cotton_array_ = array_transforms
            return

        target_index = self._obtainTargetIndex(target_row, target_col)

        if not self._isValidIndex(target_index, len(point_list)):
            rospy.logwarn(f"Target index {target_index} is out of range")
            self.tf_cotton_array_ = array_transforms
            return

        x, y, z = point_list[target_index]

        if self._isAnyNan(x, y, z):
            newXYZ = self._handleNanPoints(point_list, target_index) #Override x,y,z values
            if np.array_equal(newXYZ, np.zeros(3)):
                return
            x,y,z = newXYZ

        self._logInfoPoint(idx, target_row, target_col, x, y, z)

        t = self._createTransform(idx, x, y, z)
        #array_transforms.append(t)

        self.tf_cotton_array_ = t
        
    def getDepthCentroid3d(self):
        #Call function point cloud
        self.callback_pointCloud()
        return self.tf_cotton_array_