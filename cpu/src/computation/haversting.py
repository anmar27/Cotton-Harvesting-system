#!/usr/bin/env python3

#Generic libraries
from __future__ import print_function
from statemachine import State, StateMachine
from typing import Any
import torch
from queue import PriorityQueue
import tf.transformations
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
import tf2_ros
import tf2_geometry_msgs
import math
import tf
import random
import copy

## ROS
import rospy
import sys


from std_msgs.msg import (
    String as ROSString
)
from sensor_msgs.msg import (
    Image as ROSImage
)

## Our code
#from .world_state import WorldState, CottonData
from computation.cpu import CPUNode

from geometry_msgs.msg import TransformStamped, Quaternion

#Libraries diagram 
from statemachine.contrib.diagram import DotGraphMachine

# .srv
from computation.srv import TFpublisher
from computation.srv import CartesianMovement, CartesianMovementResponse

#Definition state machine
class DemeterLogic(StateMachine):

    """
    Initial state 
    """
    stand_by = State(name="stand_by",initial=True)

    """
    States from exploration
    """
    # States from Exploration state
    #Explore = State       # Has to be changed when the exploration log is implemented
                            # TODO Explore state has to have ane xit action that returns true or false, whether it has found a cotton to harvest

    """
    States from cotton harvesting
    """
    ## Current states
    # TODO add callbacks for every time it enters/stays/exits a state (precondition, operate, postcondition and action returning)
    ready_harvest = State(name="ready_harvest")
    scene_mapped = State(name="scene_mapped")
    pre_arm_placed = State(name="pre_arm_placed")
    arm_placed = State(name="arm_placed",final=True)
    #grab_cotton = State(name ="grab_cotton")

    """
    End the state machine execution
    """
    #end = State(name="end",final=True)


     ## Transitions
    # TODO transformar transiciones básicas a cosas más complejas, con precondiciones, 
    # funciones a ejecutar antes/durante/después de aplicar el estado, etc...

    #stay_still = stand_by.to(stand_by)              # Transition -> Wait for initialization
    #init = stand_by.to(Explore)                     # Transition -> Start the exploring

    #cotton_found = Explore.to(ready_harvest)        # Transition -> Enter the cotton harvesting (TODO when Explore is implemented, has to be changed to smaller state)
    starting_haversting = stand_by.to(ready_harvest)
    map_scene = ready_harvest.to(scene_mapped)      # Transition
    approach_pre_aux_frame = scene_mapped.to(pre_arm_placed)  # Transition
    error_on_approach = pre_arm_placed.to.itself() #Transition -> Error on pre_arm placing
    approach_aux_frame = pre_arm_placed.to(arm_placed)
    
    #get_closer_segmentation = arm_placed.to.
    #true_cotton = arm_placed.to(grab_cotton)        # Transition
    #cotton_harvested = grab_cotton.to(scene_mapped) # Transition
    #not_cotton = arm_placed.to(scene_mapped)        # Transition
    #no_cotton_left = scene_mapped.to(Explore)       # Transition -> get out of cotton harvesting

    #end_exploration = Explore.to(end)

    def __init__(self,haversting_node):
        """
        State machine constructor.

        Every object is instanciated with an empty value (0, None, false...) because the transition action 'on_init' is the one
        responsible to load properly those values.
        """
        #ROS attributes 
        self.haversting_node = haversting_node

        # Node name
        self.id: str = "DEMETER"

        # Subscribers to ROS topics

        ## Azure Kinect DK
        self.image_topic: str = ""
        self.image_sub: rospy.Subscriber = None  # rospy.Subscriber()
        self.image_buffer: list = []

        self.depth_topic: str = ""
        self.depth_sub: rospy.Subscriber = None  # rospy.Subscriber()
        self.depth_buffer: list = []

        self.cotton_static_broadcaster = StaticTransformBroadcaster()
        self.aux_static_broadcaster = StaticTransformBroadcaster()
    
        # TODO

        # Publishers to ROS topics
        # TODO

        # Grounding DINO + SAM functionalities (Computation Node)
        self.vision: CPUNode = None
        # World state of previous executions
        # This includes ids and position of cotton bolls, error estimations, and other parameters 
        # (save/load in local filesystem done by the class, nothing to worry here)
        """
        self.world: WorldState = None
        """
        # Own attributes
        """
        self.cotton_order: PriorityQueue = None
        self.cur_cotton: CottonData = None
        """

        """ Robot configurations """

        self.initial_pose_harvest: tuple = ()   # Eventually this attribute will change to something that saves the configuration of 
                                                # the robot at the exact same point where it starts harvesting (harvest_cotton state)

        """ END OF ATTRIBUTES """

        # Parent class initialization
        super().__init__()
    
    ####### STATE ASSOCIATED FUNC.#######

    ### Aux. fucn. scene_mapped state ###
    def concatenate_tf_robotBase_aux(self,base_to_cotton_tf,translation_x,translation_y,translation_z,q1_list,q2_list):

        #Concatenation of translation 
        robotbase_to_aux_transform_Z_translation = [base_to_cotton_tf[0][0] + translation_z,
            base_to_cotton_tf[0][1] + translation_x,
            base_to_cotton_tf[0][2] - translation_y,
        ]

        #Concatenation of rotation
        robotbase_to_aux_transform_Z_rotation = tf.transformations.quaternion_multiply(q1_list,q2_list)

        return robotbase_to_aux_transform_Z_translation, robotbase_to_aux_transform_Z_rotation

    def sending_static_transforms(self,list_transforms):
        self.aux_static_broadcaster.sendTransform(list_transforms)
        ##Sending static transforms in batches
        #    for i in range(0, len(list_transforms), 5):
        #        batch = list_transforms[i:i + 5]
        #        self.aux_static_broadcaster.sendTransform(batch)
            

    def populate_transform(self,trans,total_count,translation,rotation):
        #Creation of tf from robot_base to auxiliar_frame
        trans.header.frame_id = "robot_base"
        trans.child_frame_id = "aux_frame_"+str(total_count)

        trans.transform.translation.x = translation[0]
        trans.transform.translation.y = translation[1]
        trans.transform.translation.z = translation[2]

        trans.transform.rotation.x = rotation[0]
        trans.transform.rotation.y = rotation[1]
        trans.transform.rotation.z = rotation[2]
        trans.transform.rotation.w = rotation[3]
    
    def generateTFsOverVerticalAxis(self,z_vector,angle_z,subset,dist,res_tf):
        transforms = []
        tf_listener = tf.TransformListener()
        robotBase_aux_transform_Z = TransformStamped()
        count = 1+(subset*len(z_vector))
        
        for angles in range(len(z_vector)):
            #Translation 
            translation_x = -dist*math.sin(angle_z*(math.pi/180)) #-dist*math.sin(z_vector[angles]*(math.pi/180)) #Keeped fixed 
            translation_y = -dist*math.sin(z_vector[angles]*(math.pi/180))
            translation_z = -dist*math.cos(angle_z*(math.pi/180))
            #translation_z = -dist*math.cos(z_vector[angles]*(math.pi/180))
            #Rotation
            roll = -z_vector[angles]*(math.pi / 180.0)
            pitch = -angle_z*(math.pi/180.0)
            yaw = 0
            q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
            while not tf_listener.canTransform(res_tf.transform.child_frame_id,"robot_base",rospy.Time().now()):
                rospy.loginfo("Waiting for transform robot_base -> cotton")
                rospy.sleep(1)
                       
            robotbase_to_cotton_transform = tf_listener.lookupTransform("robot_base",res_tf.transform.child_frame_id,rospy.Time().now())
            print("Robot Base->Cotton",robotbase_to_cotton_transform)
               
            quaternion_cotton_aux = [q[0],q[1],q[2],q[3]]
            quaternion_robotbase_cotton = [-0.5,0.5,-0.5,0.5]
            robotbase_to_aux_transform_Z_translation, robotbase_to_aux_transform_Z_rotation = self.concatenate_tf_robotBase_aux(robotbase_to_cotton_transform,
                                                                                                                                   translation_x,
                                                                                                                                   translation_y,
                                                                                                                                   translation_z,
                                                                                                                                   quaternion_robotbase_cotton,
                                                                                                                                   quaternion_cotton_aux)
            # Create a copy of robotBase_aux_transform_Z before modification
            transform = copy.deepcopy(robotBase_aux_transform_Z)
            self.populate_transform(transform,
                                    count,
                                    robotbase_to_aux_transform_Z_translation,
                                    robotbase_to_aux_transform_Z_rotation)
            
            transforms.append(transform)
            count += 1 
        print("Sucessfully created TFs over the z axis with a rotation of", angle_z,"radians")
        self.sending_static_transforms(transforms)
        return transforms
    ####### scene_mapped state #######

    @scene_mapped.enter
    def enter_scene_mapped(self) -> bool:
        """
        Method to execute when in scene_mapped state (execution of the state)
    
        TODO:
            - Retrieve tf of real-world positions of the different cottons on the scene
        Procedimiento:
        """
        res_tf = self.haversting_node.retrieve_cotton_transforms(1,"camera_frame")
        

        if res_tf:
            
            dist_cotton_aux = 0.35 #in meters

            #Tf listener 
            tf_listener = tf.TransformListener()
            robotBase_aux_transform_Z = TransformStamped()
            list_transforms = []

            #Creation of the cotton frame itself
            
            print("Transform received:", res_tf)
            print(res_tf.transform.child_frame_id)
            try:
                self.cotton_static_broadcaster.sendTransform(res_tf.transform)
            except:
                rospy.logwarn("Failed to send transform cotton_frame to rgb_link")

            #rospy.sleep(2) #Not ideal, should be waitForTransform
            #tf_listener.waitForTransform(res_tf.transform.child_frame_id,"robot_base",rospy.Time().now(),rospy.Duration(5.0))
            
            z_angles = [-30,-15,0,15,30]
            z_rotation_angles = [-90,-75,-60,-45,-30,-15,0,15,30,45,60,75,90]

            for subset in range(len(z_rotation_angles)):
                print(subset)
                #breakpoint()
                transforms = self.generateTFsOverVerticalAxis(z_angles,z_rotation_angles[subset],subset,dist_cotton_aux,res_tf)
                #breakpoint()
                self.cotton_static_broadcaster.sendTransform(res_tf.transform)
                list_transforms = list_transforms + transforms
            '''
            #Creation of auxiliuary cotton approachs frames (25 in this case)
            list_angles = [30,15,0,-15,-30]
            iterations = [9, 7, 5, 3, 1] # number of aux frames sum(vertial+horizontal)
            mod_list_angles = list_angles.copy()
            count = 0
            total_count = 0
            
            for aux_frame in iterations: #Iterate through 5 subsets
                #Consider first case
                if count != 0:
                    mod_list_angles = list_angles[count:]
                
                for subset in range(aux_frame): # 9,7,5,3,1
                    dimension = len(mod_list_angles)
                    if subset < dimension: # "X axis" (Must be distinction between axis) 
                        #Translation 
                        translation_x = -dist_cotton_aux*math.sin(list_angles[count]*(math.pi/180)) #Keeped fixed 
                        translation_y = -dist_cotton_aux*math.sin(mod_list_angles[subset]*(math.pi/180))
                        translation_z = -dist_cotton_aux*math.cos(mod_list_angles[subset]*(math.pi/180))
                        #Rotation
                        roll = -mod_list_angles[subset]*(math.pi / 180.0)
                        pitch = -mod_list_angles[0]*(math.pi/180.0)
                        yaw = 0
                        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
                    else:   #"Y axis"
                        #Translation 
                        print("Set",count)
                        print("Subset",subset)
                        translation_x = -dist_cotton_aux*math.sin(mod_list_angles[subset-dimension+1]*(math.pi/180))
                        translation_y = -dist_cotton_aux*math.sin(list_angles[count]*(math.pi/180))
                        translation_z = -dist_cotton_aux*math.cos(mod_list_angles[subset-dimension+1]*(math.pi/180))
                        #Rotation
                        roll = -mod_list_angles[0]*(math.pi / 180.0)
                        pitch = -mod_list_angles[subset-dimension+1]*(math.pi/180.0)
                        
                        yaw = 0
                        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
                    
                    while not tf_listener.canTransform(res_tf.transform.child_frame_id,"robot_base",rospy.Time().now()):
                        rospy.loginfo("Waiting for transform robot_base -> cotton")
                        rospy.sleep(1)
                        
                    robotbase_to_cotton_transform = tf_listener.lookupTransform("robot_base",res_tf.transform.child_frame_id,rospy.Time().now())
                    print("Robot Base->Cotton",robotbase_to_cotton_transform)
                    
                    quaternion_cotton_aux = [q[0],q[1],q[2],q[3]]
                    quaternion_robotbase_cotton = [-0.5,0.5,-0.5,0.5]

                    robotbase_to_aux_transform_Z_translation, robotbase_to_aux_transform_Z_rotation = self.concatenate_tf_robotBase_aux(robotbase_to_cotton_transform,
                                                                                                                                        translation_x,
                                                                                                                                        translation_y,
                                                                                                                                        translation_z,
                                                                                                                                        quaternion_robotbase_cotton,
                                                                                                                                        quaternion_cotton_aux)
                    # Create a copy of robotBase_aux_transform_Z before modification
                    trans = copy.deepcopy(robotBase_aux_transform_Z)
                    self.populate_transform(trans,
                                            total_count,
                                            robotbase_to_aux_transform_Z_translation,
                                            robotbase_to_aux_transform_Z_rotation)
                    list_transforms.append(trans)
                    rospy.loginfo("Added static auxiliary frame %d",total_count)
                    total_count += 1
                count += 1
            '''
            self.sending_static_transforms(list_transforms)
            self.approach_pre_aux_frame()
            
            return True
        else:
            print("Failed to receive transform")
            return False

        

    @scene_mapped.exit
    def exit_scene_mapped(self) -> None:
        """
        Method to execute when exiting scene_mapped state (checking of postconditions / others)
        """
        rospy.loginfo("Exiting scene mapped state")
        #Gettign rid of transform 
        #if hasattr(self,'cotton_static_broadcaster'):
        #    self.static_broadcaster.unregister()
        #    del self.static_broadcaster

    ### Aux. fucn. arm_placed state ###
    def tuple_to_transformstamped(self,data, header_frame_id, child_frame_id):
        """
        Converts a tuple representing a transform (translation, rotation quaternion) to a TransformStamped message.

        Args:
            data: A tuple containing (translation, rotation quaternion) as two lists of length 3 and 4 respectively.
            header_frame_id: String, frame ID of the origin of the transform (default: "map").
            child_frame_id: String, frame ID of the target of the transform (default: "base_link").
            timestamp: rospy.Time object specifying the timestamp for the transform (default: None, uses current time).

        Returns:
            A geometry_msgs/TransformStamped message representing the input transform.
        """
        try:
            transform = TransformStamped()
            transform.header.frame_id = header_frame_id
            transform.child_frame_id = child_frame_id

            transform.header.stamp = rospy.Time.now()

            # Unpack translation and rotation from the tuple
            translation, rotation = data

            # Set translation
            transform.transform.translation.x = translation[0]
            transform.transform.translation.y = translation[1]
            transform.transform.translation.z = translation[2]

            # Set rotation as quaternion
            transform.transform.rotation.x = rotation[0]
            transform.transform.rotation.y = rotation[1]
            transform.transform.rotation.z = rotation[2]
            transform.transform.rotation.w = rotation[3]
        except(IndexError,TypeError):
            print("tuple_to_transformsstamped_Error: Invalid data format for transform tuple.")
            print("Please verify that you specified the correct frames")
            return None
        
        return transform

    ####### arm_placed state #######

    @pre_arm_placed.enter
    def enter_pre_arm_placed(self) -> bool:
        """
        Method to execute when in pre_arm_placed state (execution of the state)

        TODO:
            - Nos movemos justo delante del algodón, a una distancia prudencial, de manera que lo observemos bien y podamos recogerlo
                (habría que mirar la pose del algodón para determinar la normal y pillarlo desde ahí?) --> TODO interesante acercamiento
            
        """
        #Creation func. variables
        tf_listener = tf.TransformListener()

        #Approach random aux.frame
        #aux_frame_selected = random.randint(0,24)
        aux_frame_selected = 15
        aux_frame_str = "aux_frame_" + str(aux_frame_selected)
        print("Moving towards auxilary frame number", aux_frame_selected)


        while not tf_listener.canTransform(aux_frame_str,"base_link",rospy.Time().now()):
            rospy.loginfo("Waiting for transform aux_frame -> base_link")
            rospy.sleep(1)

        base_link_to_aux_transform = tf_listener.lookupTransform("base_link",aux_frame_str,rospy.Time().now())
        print(base_link_to_aux_transform)

        transform_msg = self.tuple_to_transformstamped(base_link_to_aux_transform,header_frame_id="base_link",child_frame_id=aux_frame_str)
        if transform_msg is None:
            print("Failed to execute conversion in " + str(sm.current_state.id))
        else:
            sucess_movement = self.haversting_node.send_success_aproach_movement(transform_msg)

        if sucess_movement:
            #Calling again image for segmentation
            print("Correct Movement")
        else:
            #Trying with another aux frame
            self.error_on_approach()
        

        #Transform world->aux tf in geometry_msgs

        # TODO
        

    @pre_arm_placed.exit
    def exit_pre_arm_placed(self) -> None:
        """
        Method to execute when exiting pre_arm_placed state (checking of postconditions / others)

        TODO:
            - Determinamos si es un algodón a cosechar. Si es así devolvemos true
        """
        # Es cosechable?
        return True
        # No es cosechable?
        return False


class HaverstingNode():
    '''
    Node which manages communication with external nodes
    '''

    def __init__(self) -> None:
        print("StateMachine Node constructor launched")

        #Client to tf_publisher
        rospy.wait_for_service('tf_cotton_provider')
        self.tf_handle = rospy.ServiceProxy('tf_cotton_provider',TFpublisher)

        rospy.loginfo("Pre service")
        #Service client provider of cartesian coordinates for approach to cotton
        rospy.wait_for_service('/pre_arm_placed/cartesian_coord')
        rospy.loginfo("Post service")
        self.cartesianCoord_client = rospy.ServiceProxy('/pre_arm_placed/cartesian_coord', CartesianMovement)

    #Method which retrieves tf from the wanted cotton bolls
    def retrieve_cotton_transforms(self,number_cotton_tf: int ,camera_frame: str) -> TransformStamped:
        #returns the tf_response tf tranform
        try:
            print("Provided parameters to tf_cotton_service: " + str(number_cotton_tf) + " "+ str(camera_frame))
            response = self.tf_handle(number_cotton_tf,camera_frame)    
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def send_success_aproach_movement(self,transform):

        try:
            response = self.cartesianCoord_client(transform)
            return response
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            rospy.sleep(1)
            return False
        
# Main functionality

if __name__ == '__main__':

    try:
        rospy.init_node("sm_haversting_node", anonymous=True)
        harversting_node = HaverstingNode()
        sm = DemeterLogic(haversting_node=harversting_node)
        print(sm.current_state)
        while not rospy.is_shutdown():
            if sm.current_state == sm.stand_by:
                sm.starting_haversting()
                sm.map_scene()
            
        graph = DotGraphMachine(sm)
        graph().write_png("/iri_lab/iri_ws/exampleSM.png")
    except rospy.ROSInterruptException:
        pass
