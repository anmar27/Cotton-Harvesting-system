"""
Localizaiton SuperState for DEMETER. This class constitutes the state 'Localization' of the flow chart.
"""

# Generic modules
import rospy
import tf
import tf2_ros

# State machine
from smach import State, StateMachine
from smach.user_data import UserData

# Own modules
from utils.utils_sm import *

from utils.vision import DepthLocalization
from utils.utils_tf import CottonFrameBroadcaster,FixedFrameBroadcaster, generationNetTFs
from geometry_msgs.msg import TransformStamped
import std_srvs.srv

class CottonBollDepth(State):

    def __init__(self):
        State.__init__(self,outcomes = [COTTON_BOLL_DEPTH_END],
                         input_keys=['centroid2D','point_cloud'],
                         output_keys=[COTTON_BOLL_DEPTH_OUTPUT])
        
    def execute(self, userdata):
        """
        - Based on the position of Centroid in the Image coordinate system (2D) translantes that to 
        Camera coordinate system
        """
        print("Inside cototonDepth")
        centroid = userdata.centroid2D
        pointcloud = userdata.point_cloud
        self.depthLocalization = DepthLocalization(centroid,pointcloud)
        
        while True:
            cotton_TF_depthLink = self.depthLocalization.getDepthCentroid3d() 
            if cotton_TF_depthLink == None:
                rospy.loginfo("Not depth info yet!!")
                rospy.sleep(0.5)
            else:
                break
        userdata.cotton_depth_output = cotton_TF_depthLink
        print("DEPTH")
        print(cotton_TF_depthLink) 
        return COTTON_BOLL_DEPTH_END

class CottonWorldCoordinate(State):
    """
        - Based on position on the camera coordinate system translate it to
        World coordinates
    """
    #TODO -> by the moment camera and world coordinate system is cosidered equal, therefore same origin
    #TODO -> Put all logic inside util instead of state itself
    def __init__(self):
        State.__init__(self,outcomes = [COTTON_WORLD_COORDINATES_END,
                                        "remain_detection"],
                         input_keys=[COTTON_WORLD_COORDINATES_INPUT],         
                         output_keys=[COTTON_WORLD_COORDINATES_OUTPUT])
        self.publisher = rospy.Publisher('/cotton_tf/camera_H_cotton', TransformStamped, queue_size=1) #Assuming max of 5 cottons per frame 
        
    def execute(self, userdata):
  
        # TODO -> Integrate camera_depth_Transform_world transformation
        cotton_T_camerauserdata = userdata.cotton_world_coordinates_input
        self.cottonBroadcaster = CottonFrameBroadcaster(cotton_T_camerauserdata)
        print("TFFFFF")
        print(cotton_T_camerauserdata)
        userdata.cotton_world_coordinates_output = userdata.cotton_world_coordinates_input 
        self.publisher.publish(cotton_T_camerauserdata)

        #Populating auxiliar TFs
        aux_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        total_transforms = []
        
        #Obtain values from parameters
        dist_cotton_aux = rospy.get_param("/detection_parameters/distance_recheck") #in meters
        z_angles = rospy.get_param("/detection_parameters/z_angles")
        z_rotation_angles = rospy.get_param("/detection_parameters/z_rotation_angles")

        cotton_tf = "cotton_frame_0"
        tf_listener = tf.TransformListener()
        while not tf_listener.canTransform(cotton_tf,"base_link",rospy.Time(0)):
                rospy.loginfo("Waiting for transform base-link -> cotton")
                rospy.sleep(1)
                       
        robotbase_to_cotton_transform = tf_listener.lookupTransform("base_link",cotton_tf,rospy.Time(0))
        print("Robot Base-Link->Cotton", robotbase_to_cotton_transform)

        #Instance of creation for auxiliar frames
        self.creator_auxiliar_frames = generationNetTFs(robotbase_to_cotton_transform)

        for subset in range(len(z_rotation_angles)): #Interate over vertical axes
            transforms_one_vertical_axis = self.creator_auxiliar_frames.generateTFsOverVerticalAxis(z_angles,
                                                                                                    z_rotation_angles[subset],
                                                                                                    subset,dist_cotton_aux,
                                                                                                    cotton_tf)
            for sub_sub_set in range(len(z_angles)): #append one by one the transforms -> format "sendTransform"
                total_transforms.append(transforms_one_vertical_axis[sub_sub_set])
        total_transforms.append(cotton_T_camerauserdata)
        aux_static_broadcaster.sendTransform(total_transforms)

        rospy.sleep(3)
        
        #Create service to set manipulation-ready-to-start #TODO maybe in the future needed to add the Pose info
        try:
            rospy.wait_for_service('/manipulation_ready_to_start')
            manipulation_ready_to_start = rospy.ServiceProxy('/manipulation_ready_to_start', std_srvs.srv.SetBool)
            resp = manipulation_ready_to_start(True)
            if resp.success == True:
                rospy.loginfo("Manipulation started sucessfully")
                rospy.loginfo(resp.message)
            else:
                rospy.logerr("Problem found in manipulation starting")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        return "remain_detection"
    
class Localization(StateMachine):
    """
    State Localization for DEMETER state machine. Executes everything related to cotton boll localization.
    """
    ## Constructor
    def __init__(self):
        StateMachine.__init__(self, outcomes=[EXIT_LOCALIZATION,"continue_perception"],
                              input_keys=['centroid2D','point_cloud'])
                                #,
                                #input_keys=[LOCALIZATION_INPUT],         
                                #output_keys=[LOCALIZATION_OUTPUT])

        with self:

            self.add('CottonBollDepth', 
                     CottonBollDepth(),
                     transitions={COTTON_BOLL_DEPTH_END: 'CottonWorldCoordinate'},
                     #input_keys=[CAMERA_ADJUST_OUTPUT])
                     remapping={'centroid2D':'centroid2D',
                                'point_cloud':'point_cloud'})
            
            self.add('CottonWorldCoordinate', 
                     CottonWorldCoordinate(),
                     transitions={COTTON_WORLD_COORDINATES_END:EXIT_LOCALIZATION,
                                  "remain_detection":"continue_perception"},
                     remapping={COTTON_WORLD_COORDINATES_INPUT: COTTON_BOLL_DEPTH_OUTPUT,
                                COTTON_WORLD_COORDINATES_OUTPUT:LOCALIZATION_OUTPUT})
            
        
    def execute(self,userdata):
        """
        Execution of state machine
        """
        rospy.loginfo("Executing Localization state machine")
        outcome = super(Localization, self).execute(userdata)
        return outcome