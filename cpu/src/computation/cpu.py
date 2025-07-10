#!/usr/bin/env python3

 #
    #El workflow es el siguiente:
#
    #0. Continuamente recibimos (imágen, profundidad) del canal de comunicación.
    #1. En algún momemento de la ejecución, tenemos que recuperar esa tomar la última imagen y tratarla,
    #para obtener predicciones, bounding boxes, máscaras y centroides
    #2. A partir de esta información obtenemos (x,y,z) e instrucciones del siguiente estado de la máquina
    #    2.1 Máquina de estados para representar el modus operandi del robot? Qué tipo de planificador? Investigar al respecto
    #3. Mandar instrucciones a las otras partes del robot (brazo y desplazamiento principalmente)
#
    #ANOTACIONES:
#
    #A. Quizá nos podemos basar en el ciclo BDI de los sistemas distribuidos (beliefs, desires, intentions)
    #B. Algoritmos de planificación: Buscar, cuáles hay, en qué están basados.


#Libraries/modules
import pdb
import rospy
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
from PIL import Image
import torch
from torchvision import transforms
import cv2
from cv_bridge import CvBridge, CvBridgeError
#import centroid  #Add path to PYTHONPATH $HOME/Desktop/Demeter-5.0/demeter-5.0/src
from centroid import Centroid
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# Types of return of Azure Kinect ROS node (rgb/image_raw, depth_to_rgb/image_raw)
from sensor_msgs.msg import (
    CameraInfo as ROSCameraInfo, 
    Image as ROSImage,
)

#Add wrapper of Azure Kinect SDK
#import pykinect_azure as pykinect

#Libraries for srv messages
import tf
from geometry_msgs.msg import TransformStamped

# .srv
from computation.srv import TFpublisher, TFpublisherResponse

#GLOBAL VARIABLES (Transform it to parameters later)
WINDOWS_OR_CENTROIDS = False
SEGMENTATION = False #True -> Also obtain SAM mask
WIDTH = 1280
HEIGHT = 720
MODEL = "noyolo"


class CPUNode():
    """
    ROS Node that computes all necessary operations and computes planification algorithms for other components.

    This is the main logic and computational unit to operate the robot. By the moment, it's the only one, maybe in the 
    future we can divide and parallelise the execution to obtain better results.
    """
    # Class methods

    def __init__(self) -> None:
        self.aproxCentroidsImage_: ROSImage = None
        self.depthImage_ :ROSImage = None
        self.aprox_centroids_values_ : np.ndarray = []
        self.masks_ : np.ndarray = []
        self.tf_cotton_array_ = []

        self.bridge_ = CvBridge()
        self.centroid_ = Centroid(MODEL)


        #Initialized param
        rospy.set_param('/cpu_node_initialized',True)

        #Add wait for services

        #Create Subscriptions to camera topics
        if SEGMENTATION:
            rospy.Subscriber('rgb/image_raw',ROSImage, callback=self.callback_SAM,queue_size=1)
        else:
            rospy.Subscriber('rgb/image_raw',ROSImage, callback=self.callback_DINO, queue_size=1)
        
        rospy.Subscriber('depth_to_rgb/image_raw',ROSImage, callback=self.callback_depthImage, queue_size=1)
        rospy.Subscriber('points2', PointCloud2, callback=self.callback_pointcloud, queue_size=10)
        

        #Creation of Publishers

        self.mod_image_pub = rospy.Publisher('rgb/mod_image_raw', ROSImage, queue_size=50)
        self.depth_image_pub = rospy.Publisher('depth/cv_image', ROSImage, queue_size=50)

        #Creation of services

        cotton_tf = rospy.Service('tf_cotton_provider', TFpublisher, self.handle_tf_cotton_provider)


    #Getters
    @property
    def get_aprox_centroids(self):
        return self.aprox_centroids_values_
    
    # Utils

    @staticmethod
    def rosImage2tensor(self, ros_image: ROSImage) -> torch.Tensor:
        """
        Given an image in ROS message format, return it in torch-like format

        In the repo, it is said that it's in format BGRA
        """

        try:
            cv_image = self.bridge_.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError as err:
            print(err)

        # Convert OpenCV image to PyTorch tensor
        transform = transforms.ToTensor()
        tensor_image = transform(cv_image)
        
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)

        # If the image is in BGRA format (which is the case AT THE MOMENT), convert it to RGB
        if tensor_image.shape[0] == 4:
            # Uncomment this line in case the program works with (H, W, C) formatting
            # tensor_image = tensor_image.permute(1, 2, 0)  # Change from (C, H, W) to (H, W, C)
            tensor_image = tensor_image[[2, 1, 0], :, :]  # Swap B and R channels

        return tensor_image, bgr_image

    """
    <arg name="depth_mode"              default="WFOV_UNBINNED" /> Set the depth camera mode, which affects FOV, depth range, and camera resolution. 
                                        See Azure Kinect documentation for full details.
    <arg name="depth_unit"              default="16UC1" /> 16UC1 is 16-bit integer of millimeters
    """
    @staticmethod
    def rosDepthMap2tensor(self, ros_depth: ROSImage) -> torch.Tensor:
        """
        Gets a depth image (map) of the current image and returns a tensor representing this info.

        The metrics (HxW, starting point) are the same as an image.

        - TODO ver si esto último es cierto, dado que la cámara de profundidad es de 1MP, y la de imagen es de 12MP
        """
        
        cv_image = self.bridge_.imgmsg_to_cv2(ros_depth, encoding="passthrough")
        # Convert OpenCV image to PyTorch tensor
        tensor_depth = torch.from_numpy(cv_image)

        # Maybe we need a transformation of tensor (at the moment, the tensor is a matrix of 16UC1 elements, HxW)

        return tensor_depth

    # Functionalities

    def image_callback(self, data):
        """
        Prints the retreived data
        """
        tensor_image = CPUNode.rosImage2tensor(data)    # This function gets Image, not Image.data
        self.image_buffer.append(tensor_image)
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def depth_callback(self, data):
        """
        Prints the retreived data
        """
        tensor_depth = CPUNode.rosDepthMap2tensor(data)
        self.depth_buffer.append(tensor_depth)
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def callback_DINO(self,data): #rgb/image_raw callback (5Hz)

        print("EEE")
        self.testImage = data

        tensor_img, cv_image = self.rosImage2tensor(self,data) 
        #Apply DINO model
        aprox_centroids, execution_time, cv_image_points, reception_flag = self.centroid_.centroids_process_single_imageROS(tensor_img,cv_image,WINDOWS_OR_CENTROIDS,SEGMENTATION)
        #print(aprox_centroids)
        #print(execution_time)
        
        self.aprox_centroids_values_ = aprox_centroids

        if reception_flag == 0:
           rospy.loginfo(rospy.get_caller_id() + " Did not found matches in this frame")
           self.aproxCentroidsImage_ = self.bridge_.cv2_to_imgmsg(cv_image_points, "bgr8")
           rospy.loginfo(rospy.get_caller_id() + " Model execution time: %s", str(execution_time))

        else:
            rospy.loginfo(rospy.get_caller_id() + " Found matches in this frame")
            self.aproxCentroidsImage_ = self.bridge_.cv2_to_imgmsg(cv_image_points, "bgr8")
            rospy.loginfo(rospy.get_caller_id() + " DINO execution time: %s", str(execution_time))
            rospy.loginfo(rospy.get_caller_id() + " Model points founded: %s", str(aprox_centroids))  
    
    def callback_SAM(self,data):
        tensor_img, cv_image = self.rosImage2tensor(self,data)
        masks, execution_time, cv_image, reception_flag = self.centroid_.centroids_process_single_imageROS(tensor_img,cv_image,WINDOWS_OR_CENTROIDS,SEGMENTATION)

        self.masks_ = masks

        if reception_flag == 0:
           rospy.loginfo(rospy.get_caller_id() + " Did not found matches in this frame")
           self.aproxCentroidsImage_ = self.bridge_.cv2_to_imgmsg(cv_image, "bgr8")
           rospy.loginfo(rospy.get_caller_id() + " Model execution time: %s", str(execution_time))

        else:
            rospy.loginfo(rospy.get_caller_id() + " Found matches in this frame")
            self.aproxCentroidsImage_ = self.bridge_.cv2_to_imgmsg(cv_image, "bgr8")
            rospy.loginfo(rospy.get_caller_id() + " Model execution time: %s", str(execution_time))
            rospy.loginfo(rospy.get_caller_id() + " Number of masks founded: %s", str(len(masks)))

    
    def callback_depthImage(self,data): #depth_to_rgb/image_raw callback (5Hz)
        depth_image_cv = self.bridge_.imgmsg_to_cv2(data,"passthrough")  #if depth/image_raw required to implement transformation kinect SDK

        if SEGMENTATION: #Depending if want median depth of the object or simple dist to aprox_centroid (this last one is 5 times faster )
            depth_each_object = []

            for mask in self.masks_:
                mask = mask.squeeze(0) #Leave only 720x1280 image
                true_indices = torch.nonzero(mask, as_tuple=False)
                #print(true_indices)
                mask_depth_values = [0] * true_indices.size()[0]
                for i in range(true_indices.size()[0]) :
                    mask_depth_values[i] = depth_image_cv [true_indices[i][0].item()] [true_indices[i][1].item()]
                
                
                depth_each_object.append(mask_depth_values[true_indices.size()[0]//2])
                
            if depth_each_object == []:
                pass
            else:
                print("Masks Centroids: "+ str(depth_each_object))
            
            self.depthImage_ = data 

        else:

            centroids = self.aprox_centroids_values_
            print("Found: " + str(len(centroids)) + " centroids")
            for centroid in centroids:   #Consider case of too close camera to object (Range : 0.5-3 m)
                depth_value_centroid = depth_image_cv[centroid[1],centroid[0]] #First y sencond x
                print("Centroid Distance: "+ str(depth_value_centroid) + " mm")
                print("Pixel: " + str(centroid))
            self.depthImage_ = data
    
    def callback_pointcloud(self,data):  #Obtains the x,y,z values of centroids 
        idx = 0
        array_tranforms = []
        
        
        #assign centroid pixel indices (Please note that x and y are swapped)
        target_rows = [item[1] for item in self.aprox_centroids_values_]
        target_cols = [item[0] for item in self.aprox_centroids_values_]

        points = pc2.read_points(data,field_names=("x","y","z")) # In case of desire point is Nan remove it ?
        
        point_list = list(points)
        #print("Length of pointCloud : " + str(len(point_list)))
        
        # Iterate over the lists of target rows and columns + check if is out of borders
        for target_row, target_col in zip(target_rows, target_cols):
            if target_row < HEIGHT and target_col < WIDTH:
                target_index = target_row * WIDTH + target_col #Linear index

                # Check if the target index is within the range
                if target_index < len(point_list):
                    idx += 1
                    x, y, z = point_list[target_index]
                    rospy.loginfo(f"Point at ({target_row}, {target_col}): x={x}, y={y}, z={z} in meters")
                    child_string = "cotton_frame_" + str(idx)

                    t = TransformStamped()
                    t.header.stamp = rospy.Time().now()
                    t.header.frame_id = 'rgb_camera_link'
                    t.child_frame_id = child_string
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.translation.z = z

                    #Fix rotation only consider 1.3 degrees of tilt in X axis of 'depth_camera_link' on WFOV
                    q = [0,0,0,1]
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    array_tranforms.append(t)

                else:
                    rospy.logwarn(f"Target index {target_index} is out of range")
            else:
                rospy.logwarn(f"Target row {target_row} or column {target_col} is out of bounds")
        self.tf_cotton_array_ = array_tranforms


    def pubModImage(self, event=None):
        if self.aproxCentroidsImage_ is not None:
            #print("KKKKKKKK")
            #print(str(self.aproxCentroidsImage)[:180])
            self.mod_image_pub.publish(self.aproxCentroidsImage_)
    
    def pubDepthImage(self, event=None):
        if self.depthImage_ is not None:
            #print("JJJJJJJJ")
            #print(str(self.testImage)[:180])
            self.depth_image_pub.publish(self.depthImage_)


    def handle_tf_cotton_provider(self,request):   #Handle the response -> TF to all cottons
        try:
            array_transforms = self.tf_cotton_array_
            
            if request.number_tf == 1: #Lets assume we get the cotton with higher value
                print("Retrieve first cotton")
                
                response = TransformStamped()
                response = array_transforms[1]
                #response.transform = array_transforms[0].transform
                #response.header.frame_id = array_transforms[0].header.frame_id
                #response.child_frame_id = array_transforms[0].child_frame_id
                #response.header.stamp = array_transforms[0].header.stamp

                print(type(response))
                print(response)
                return response #return transform of cotton
            else:
                pass
            
                return TFpublisherResponse(request.camera_frame) #return transform of cotton
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")
            return TFpublisherResponse("ERROR")

#Creation of fix tf with respect to rgb_camera_link
class FixedFrameBroadcaster():
    """
    This node publishes child static transform with respect to camera_base
    """
    def __init__(self):
        """
        A function that creates a broadcast node and publishes three new transform
        frames.
        :param self: The self reference.
        """
        self.br = StaticTransformBroadcaster()

        self.robot_base = TransformStamped()
        self.robot_base.header.stamp = rospy.Time.now()
        self.robot_base.header.frame_id = 'camera_base'
        self.robot_base.child_frame_id = 'robot_base'
        self.robot_base.transform.translation.x = 0.0
        self.robot_base.transform.translation.y = 0.0
        self.robot_base.transform.translation.z = 1 #By the moment assume is static moved down 1 meter
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.robot_base.transform.rotation.x = q[0]
        self.robot_base.transform.rotation.y = q[1]
        self.robot_base.transform.rotation.z = q[2]
        self.robot_base.transform.rotation.w = q[3]

        self.br.sendTransform([self.robot_base, self.lift, self.wrist])

        rospy.loginfo('Publishing robot_base frame. Use RViz to visualize')

# Main functionality

if __name__ == '__main__':

    try:
        rospy.init_node("cpu_node", anonymous=True)
        cpu = CPUNode()
        rospy.Timer(rospy.Duration(1),cpu.pubModImage)
        rospy.Timer(rospy.Duration(1),cpu.pubDepthImage)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()