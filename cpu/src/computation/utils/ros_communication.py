"""
ROS communicator. Implements a class to request ROS data from topics, as well as TFs,
and adapts the data received to be returned to formats needed by the DEMETER logic.
"""

# Generic modules
import logging      # Can be changed to pdb if necessary

# ROS modules
import rospy
## Transforms
import tf
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import(
    Image as ROSImage,
    CameraInfo
)

# Own modules
from utils_cv import (
    rosImage2cv2image,
    rosDepth2cv2
)

# ROS communicator class
class ROSCommunicator(object):

    # Private methods
    ## Callbacks

    def __info_callback(self, data: CameraInfo):
        """
        Callback for ROS camera info data.
        """
        self.cam_info_value = data
        return
    
    def __rgb_callback(self, data: ROSImage):
        """
        Callback for ROS RGB camera data.

        TODO how is this flow going to work?
        """
        self.cam_rgb_value = rosImage2cv2image(data)
        return

    def __depth_callback(self, data: ROSImage):
        """
        Callback for ROS depth camera data.

        TODO how is this flow going to work?
        """
        self.cam_depth_value = rosDepth2cv2(data)  # Not sure about this
        return
    
    ## Start communication
    def __start_comm(self):
        """
        Starts the communication with the ROS nodes.
        """

        # Listeners
        ## Camera info listener
        self.cam_info_sub = rospy.Subscriber(self.cam_info_rostopic, CameraInfo, self.__info_callback)
        ## RGB image listener
        self.cam_rgb_sub = rospy.Subscriber(self.cam_rgb_rostopic, ROSImage, self.__rgb_callback)
        ## Depth map listener
        self.cam_depth_sub = rospy.Subscriber(self.cam_depth_rostopic, ROSImage, self.__depth_callback)

        # Subscribers

        # Others

        return
    
    ## Set static TFs
    def __set_TFs(self):
        #TODO integration already in fixedFrameBroadcaster put it here maybe
        """
        Set all initial TFs values. At the moment, we have:

        - Robot base
        - Robot camera base

        Code retrieved straight up from computation/fixedframes.py, branch develop-amartinez, 
        commit nº e0e09b38179ae13f9a9fa9eb60c7614161366b3f

        TODO every number non depending from variables has to be loaded from config.json
        """
        # Robot base
        self.robot_base.header.stamp = rospy.Time.now()
        self.robot_base.header.frame_id = 'world'
        self.robot_base.child_frame_id = 'robot_base'
        self.robot_base.transform.translation.x = 0.35
        self.robot_base.transform.translation.y = -0.0
        self.robot_base.transform.translation.z = 0.0               #By the moment assume is static moved down 1 meter
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.robot_base.transform.rotation.x = q[0]
        self.robot_base.transform.rotation.y = q[1]
        self.robot_base.transform.rotation.z = q[2]
        self.robot_base.transform.rotation.w = q[3]

        # Robot camera base
        self.robot_base_camera_base.header.stamp = rospy.Time.now()
        self.robot_base_camera_base.header.frame_id = 'robot_base'
        self.robot_base_camera_base.child_frame_id = 'camera_base'
        self.robot_base_camera_base.transform.translation.x = 0.0
        self.robot_base_camera_base.transform.translation.y = 0.0
        self.robot_base_camera_base.transform.translation.z = 0.45  #By the moment assume is static moved down 1 meter
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.robot_base_camera_base.transform.rotation.x = q[0]
        self.robot_base_camera_base.transform.rotation.y = q[1]
        self.robot_base_camera_base.transform.rotation.z = q[2]
        self.robot_base_camera_base.transform.rotation.w = q[3]

        return

    # Public methods

    def __init__(self, 
                 info_rostopic: str, 
                 rgb_rostopic: str, 
                 depth_rostopic: str) -> None:
        """
        Constructor of class:

        - Declares the needed attributes to work
        - Starts the communciation with the ROS topics
        - Starts the communication with TFs             (listener)
        - Declares initial publishing of own TFs        (publisher)
        """

        # ROS initialization
        rospy.init_node('ROSCommunicator', anonymous=False)

        # ROS topics
        ## Camera info
        self.cam_info_rostopic = info_rostopic          # Rostopic name for rgb images
        self.cam_info_sub = None                        # RGB Subscriber
        self.cam_info_value:CameraInfo = None           # RGB image at given time
        
        ## Camera image
        self.cam_rgb_rostopic = rgb_rostopic            # Rostopic name for rgb images
        self.cam_rgb_sub = None                         # RGB Subscriber
        self.cam_rgb_value: ROSImage = None             # RGB image at given time

        ## Camera depthmap
        self.cam_depth_rostopic = depth_rostopic        # Rostopic name for depthmap
        self.cam_depth_sub = None                       # Depthmap Subscriber
        self.cam_depth_value: ROSImage = None           # Depth image at given time

        # TF listener
        self.tf = tf.TransformListener()

        # Transform broadcaster for publishing
        self.br = StaticTransformBroadcaster()

        ## TransformStamped declaration for publishing (TODO)
        self.robot_base = TransformStamped()
        self.robot_base_camera_base = TransformStamped()
        self.__set_TFs()

        # Start communication
        self.__start_comm()

        return
    
    # Getters

    ## Camera info
    def get_caminfo(self):
        """
        Retrieve the current camera info from the RGB camera.
        """
        return self.cam_info_value

    ## RGB image
    def get_image(self):
        """
        Retrieve the current image from the RGB camera.
        """
        return self.cam_rgb_value

    ## Depthmap
    def get_depthmap(self):
        """
        Retrieve the current depthmap from the RGB camera.
        """
        return self.cam_depth_value

    ## TF getter
    def get_tf(self, child: str, parent: str):
        """
        Retrieve the TF from parent to child given by parameters. 
        
        The TF returned is equat to homogeneous transform H_{parent}^{child}.
        """
        try:
            (trans,rot) = self.tf.lookupTransform(child, parent, rospy.Time(0))
            return (trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            logging.debug(f"The TF from {parent} to {child} is not available")
            return (None, None)
        
    # Setters

