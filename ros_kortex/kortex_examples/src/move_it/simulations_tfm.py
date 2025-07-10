import sys
import random
import math
import csv
import os
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import tf.transformations as tf_trans
import tf
import tf2_ros
import numpy as np
from math import pi
from std_srvs.srv import Empty
import std_srvs.srv
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker

from computation.utils.utils_tf import generalUtilsTF, generationNetTFs
from util_simulations import publisher_tf_kinematics

"""
Code used to run different simulation scenarios for comparison using naive method and nbv-orient-guided method
"""

class CottonMoveItTrajectories(object):
  """CottonMoveItTrajectories"""
  def __init__(self, tf_listener):

    # Initialize the node
    super(CottonMoveItTrajectories, self).__init__()
    #rospy.init_node('cotton_move_it_trajectories')

    self.tf_listener = tf_listener
    self.is_init_success = False  # Start pessimistically
    rospy.loginfo("This is the namespace:")
    rospy.loginfo(rospy.get_namespace())
    try:
        self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
    except Exception as e:
        rospy.logerr(f"Failed to get 'is_gripper_present': {e}")
        return

    if self.is_gripper_present:
        try:
            gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
            self.gripper_joint_name = gripper_joint_names[0]
        except Exception as e:
            rospy.logerr(f"Failed to get 'gripper_joint_names' or access index 0: {e}")
            return
    else:
        self.gripper_joint_name = ""

    try:
        self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)
    except Exception as e:
        rospy.logerr(f"Failed to get 'degrees_of_freedom': {e}")
        return

    try:
        arm_group_name = "arm"
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description")
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
        self.arm_group.set_max_velocity_scaling_factor(0.99)
        self.display_trajectory_publisher = rospy.Publisher(
            rospy.get_namespace() + 'move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20
        )
    except Exception as e:
        rospy.logerr(f"Failed to initialize arm group: {e}")
        return

    if self.is_gripper_present:
        try:
            gripper_group_name = "gripper"
            self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
        except Exception as e:
            rospy.logerr(f"Failed to initialize gripper group: {e}")
            return

    rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    self.is_init_success = True


  def ik_feasability(self, target_frame):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Checking feasibility for final approach " + target_frame)
    
    transform = self.tf_listener.lookupTransform("base_link", target_frame, rospy.Time(0))
    print(transform)
    # Set the target
    pose = Pose()
    pose.position.x = transform[0][0]
    pose.position.y = transform[0][1]
    pose.position.z = transform[0][2]

    pose.orientation.x = transform[1][0]
    pose.orientation.y = transform[1][1]
    pose.orientation.z = transform[1][2]
    pose.orientation.w = transform[1][3]

    
    arm_group.set_pose_target(pose)
    arm_group.set_planning_time(10.0)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    return success_flag
    
    # Execute the trajectory and block while it's not finished
    #return arm_group.execute(trajectory_message, wait=True)

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    arm_group.set_planner_id("RRTConnect")
    #arm_group.set_planner_id("RRTstar")

    arm_group.set_planning_time(15.0) #Redefine timeout
    arm_group.num_planning_attempts = 2
    
    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 
  def transform_to_pose(self,transform):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation.x = transform.transform.rotation.x
    pose.orientation.y = transform.transform.rotation.y
    pose.orientation.z = transform.transform.rotation.z
    pose.orientation.w = transform.transform.rotation.w
    return pose

  def get_desired_tf(self,target_frame, reference_frame):
    # Initialize TF2 listener
    #tf_buffer = tf2_ros.Buffer()
    #listener = tf2_ros.TransformListener(tf_buffer)

    #while not tf_buffer.can_transform(target_frame,reference_frame,rospy.Time(0),rospy.Duration(1.0)):
    while not self.tf_listener.canTransform(target_frame, reference_frame,rospy.Time(0)):
      rospy.loginfo(f"Waiting for transform {reference_frame} -> {target_frame}")
    
    try:
        # Get the transform from the target_frame to reference_frame
        transform = self.tf_listener.lookupTransform(reference_frame, target_frame, rospy.Time(0))
        #transform = tf_buffer.lookup_transform(reference_frame, target_frame, rospy.Time(0), rospy.Duration(2.0))
        return transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Transform lookup failed{e}")
        return None
  
  #TODO -> Implement this methods in utils_tf
  def TransformStampedToTransformMatrix(self,transform_stamped):
    #print(transform_stamped[0][0])
    #translation = transform_stamped.transform.translation
    #rotation = transform_stamped.transform.rotation
    T = tf_trans.translation_matrix((transform_stamped[0][0], transform_stamped[0][1], transform_stamped[0][2]))
    R = tf_trans.quaternion_matrix((transform_stamped[1][0], transform_stamped[1][1], transform_stamped[1][2], transform_stamped[1][3]))
    H = tf_trans.concatenate_matrices(T,R)
    
    return H

  def TransformMatrixToTransformStamped(self,header_frame,child_frame,translation,rotation):
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = header_frame
    transform.child_frame_id = child_frame
    
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = rotation[0]
    transform.transform.rotation.y = rotation[1]
    transform.transform.rotation.z = rotation[2]
    transform.transform.rotation.w = rotation[3]

    return transform

  def changeEE_Pose(self, camera_frame, baseLink_transform_EE):
    rospy.loginfo(f"Changing EE to {camera_frame}")
    EE_frame = "tool_frame"

    cameraFrame_transform_EE = self.get_desired_tf(camera_frame,EE_frame)
  
    cameraFrame_H_EE = self.TransformStampedToTransformMatrix(cameraFrame_transform_EE)
    baseLink_H_EE = self.TransformStampedToTransformMatrix(baseLink_transform_EE)
    EE_H_cameraFrame = tf_trans.inverse_matrix(cameraFrame_H_EE)
    baseLink_H_cameraFrame = tf_trans.concatenate_matrices(baseLink_H_EE,EE_H_cameraFrame)
    baseLink_T_cameraFrame = tf_trans.translation_from_matrix(baseLink_H_cameraFrame)
    baseLink_R_cameraFrame = tf_trans.quaternion_from_matrix(baseLink_H_cameraFrame)

    baseLink_tranform_cameraFrame = self.TransformMatrixToTransformStamped("base_link",camera_frame,baseLink_T_cameraFrame,baseLink_R_cameraFrame)

    return baseLink_tranform_cameraFrame

  def broadcast_static_transform(self,parent_frame,child_frame):
    distance = rospy.get_param('/detection_parameters/distance_recheck')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform = TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = parent_frame  

    static_transform.child_frame_id = child_frame

    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = distance 

    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0  # Identity quaternion

    broadcaster.sendTransform(static_transform)

def go_to_home_position(tf_listener):
  manipulation_home = CottonMoveItTrajectories(tf_listener)
  
  if manipulation_home.is_init_success:
    rospy.loginfo("Sucessfully initialized the constructor routine")
  else:
    rospy.loginfo("Unsucessfully initialized the constructor routine")
  
  arm_group = manipulation_home.arm_group
  success = True
  tolerance = 0.001
  # Get the current joint positions
  joint_positions = arm_group.get_current_joint_values()
  rospy.loginfo("Printing current joint positions before movement :")
  for p in joint_positions: rospy.loginfo(p)
  # Set the goal joint tolerance
  manipulation_home.arm_group.set_goal_joint_tolerance(tolerance)
  
  # Set the joint target configuration
  if manipulation_home.degrees_of_freedom == 7:
    joint_positions[0] = 0 
    joint_positions[1] = -0.785398
    joint_positions[2] = pi
    joint_positions[3] = -pi/2
    joint_positions[4] = pi
    joint_positions[5] = 1.22173
    joint_positions[6] = -pi/2

  arm_group.set_joint_value_target(joint_positions)
  
  # Plan and execute in one command
  success &= arm_group.go(wait=True)

  if success and manipulation_home.is_gripper_present:
      success &= manipulation_home.reach_gripper_position(0)
  # Show joint positions after movement
  new_joint_positions = arm_group.get_current_joint_values()
  rospy.loginfo("Printing current joint positions after movement :")
  for p in new_joint_positions: rospy.loginfo(p)
  return success

class handle_states:
    """
    Class to handle different manipulation positions
    """
    def __init__(self, array_actual_moves, tf_listener, csv_file, csv_writer, offset_translation, default_act_cot_first_translation):
        self.tf_listener = tf_listener
        self.array_actual_moves = array_actual_moves
        self.ik_check = False
        self.start_time = None
        self.csv_file = csv_file
        self.csv_writer = csv_writer
        self.trials = 0
        self.experiment_id = 0
        self.planner = "RRTConnect"
        self.netconf = "conf_1"
        self.scenario = "scenario_1: " + str([-0.15, 0.05, 0])
        self.offset_translation = offset_translation
        self.default_act_cot_first_translation = default_act_cot_first_translation
        self.list_prev_visited_frames = []
        
        #Obtain values from parameters
        self.dist_cotton_aux = rospy.get_param("/detection_parameters/distance_recheck") #in meters
        self.z_angles = rospy.get_param("/detection_parameters/z_angles")
        self.z_rotation_angles = rospy.get_param("/detection_parameters/z_rotation_angles")
    def __generate_marker(self,position):
      pub = rospy.Publisher('visualization_marker', Marker, queue_size=1, latch=True)

      marker = Marker()
      marker.header.frame_id = "base_link"
      marker.header.stamp = rospy.Time.now()
      marker.ns = "static_marker"
      marker.id = 0
      marker.type = Marker.SPHERE
      marker.action = Marker.ADD

      # Position
      marker.pose.position.x = position[0]
      marker.pose.position.y = position[1]
      marker.pose.position.z = position[2]

      marker.pose.orientation.w = 1.0

      marker.scale.x = 0.03
      marker.scale.y = 0.03
      marker.scale.z = 0.03

      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0
      marker.color.a = 1.0

      # Lifetime of 0 = forever
      marker.lifetime = rospy.Duration(0)

      pub.publish(marker)
      
    def __obtain_projection_face_cotton(self, quaternion, translation):
      
      #Assume base frame has z axis like [0 0 1]
      z_axis_local = np.array([0, 0, 1, 0])
      rotation_matrix = tf_trans.quaternion_matrix(quaternion)
      
      z_axis_rotated = np.dot(rotation_matrix, z_axis_local)
      z_axis_rotated_3d = z_axis_rotated[:3] #Remove of homogenous vector
      
      rospy.loginfo("Z axis of cotton_frame with components: " + str(z_axis_rotated))      
      self.point_over_heatmap = z_axis_rotated_3d * self.dist_cotton_aux + translation #Cmmmon variable along class
      self.__generate_marker(self.point_over_heatmap)
      rospy.loginfo("Projection of Z cotton vector over heatmap Surface: " + str(self.point_over_heatmap))
      
    
    def __orthodromic_dist(self, lat_1, lat_2, lon_1, lon_2):
      """
      Orthodromic distance based on latitude longitude
      """
      R = self.dist_cotton_aux
      
      #convert degrees to radians
      lat1_rad = math.radians(lat_1)
      lat2_rad = math.radians(lat_2)
      dlat = lat2_rad - lat1_rad

      lon1_rad = math.radians(lon_1)
      lon2_rad = math.radians(lon_2)
      dlon = lon2_rad - lon1_rad

      #Haversine formula
      a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
      c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
      distance = R * c

      return distance
    
    def __orthodromic_distance_vector(self,A, B, center):
        """
        Calculate the orthodromic (great-circle) distance between two points on a sphere.
        """

        A = np.array(A)
        B = np.array(B)
        center = np.array(center)

        A_vec = A - center
        B_vec = B - center

        A_vec = A_vec / np.linalg.norm(A_vec)
        B_vec = B_vec / np.linalg.norm(B_vec)

        dot_product = np.clip(np.dot(A_vec, B_vec), -1.0, 1.0)  # Clip to avoid domain errors
        theta = np.arccos(dot_product)

        distance = self.dist_cotton_aux * theta

        return distance
    
    def __euclidean_distance_vector(self, A, B, center):
      """
      Calculate the Euclidean (straight-line) distance between two points in space.
      """

      A = np.array(A)
      B = np.array(B)
      center = np.array(center)

      #Shift coordinates relative to the center (optional depending on context)
      A_vec = A - center
      B_vec = B - center

      #Euclidean distance is the norm of the difference
      distance = np.linalg.norm(A_vec - B_vec)

      return distance

    def __heatmap_function_array(self,distances, k=1.0):
        distances = np.asarray(distances)  #ensure input is an array
        return 0.95 * np.exp(-k * distances)
    
    def handle_home(self,request):
      
      br_static = StaticTransformBroadcaster()
      act_cotton_frame_1 = generalUtilsTF()
      
      self.start_time = None #Reset to none & restart trials
      self.trials = 0
      self.experiment_id += 1
      self.list_prev_visited_frames = []

      #Generate actual point for cotton, giving random orientation
      rand_latitude_euler = random.randint(-60, 60)
      rand_longitude_euler = random.randint(-60, 60)
      #rand_latitude_euler = 45
      #rand_longitude_euler = 15
      
      rospy.loginfo("Random euler angles: Latitude- " + str(rand_latitude_euler) + " Longitude- " + str(rand_longitude_euler) )
  
      self.rand_latitude_euler = rand_latitude_euler * math.pi / 180
      self.rand_longitude_euler = rand_longitude_euler * math.pi / 180
  
      rand_q = tf_trans.quaternion_from_euler(self.rand_latitude_euler, -self.rand_longitude_euler , 0) #negative on y-axis
      
      self.act_cot_first_translation = [sum(x) for x in zip(self.offset_translation, self.default_act_cot_first_translation)]
      act_cot_first_rotation_z = [0, 0, 0.7071068, 0.7071068]
      act_cot_first_rotation_x = [-0.7071068, 0, 0, 0.7071068]
      act_cot_first_rotation = tf_trans.quaternion_multiply(act_cot_first_rotation_z, act_cot_first_rotation_x)
      act_cot_first_rotation = tf_trans.quaternion_multiply(act_cot_first_rotation, rand_q)
      self.__obtain_projection_face_cotton(act_cot_first_rotation, self.act_cot_first_translation)
      t_c1 = act_cotton_frame_1.publish_tf_frame(self.act_cot_first_translation, act_cot_first_rotation, parent_frame="base_link", child_frame_name="act_cotton_1")
      br_static.sendTransform(t_c1)
      
      rospy.loginfo(rospy.get_caller_id() + "Home motion requested")

      #Based if a hardcoded version is desired
      if rospy.get_param('/manipulation_parameters/hardcoded_demo') == 'yes':
        success = hardcoded_pos2()
      else: 
        success = go_to_home_position(self.tf_listener)

      if not success:
        rospy.logerr("There was an issue when trying to reach home position")
        return std_srvs.srv.TriggerResponse(False,"Problem encountered in Manipulation-Home service routine.")

      return std_srvs.srv.TriggerResponse(True,"Manipulation-Home service routine completed successfuly.")

    def handle_preapproach(self,request):

      rospy.loginfo(rospy.get_caller_id() + "Re-check motion requested")
      manipulation_re_check = CottonMoveItTrajectories(self.tf_listener)

      # For testing purposes
      success = manipulation_re_check.is_init_success
      try:
          rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
      except:
          pass

      target_frame = "test_1"  
      reference_frame = "base_link"  
    
      transform = manipulation_re_check.get_desired_tf(target_frame, reference_frame)
      #Rewrite transform to put camera_frame
      transform = manipulation_re_check.changeEE_Pose("camera_color_frame",transform)
    
      if transform:
        desired_pose = manipulation_re_check.transform_to_pose(transform)
        tolerance = 0.01  # Example tolerance in meters
        constraints = None  
        success = manipulation_re_check.reach_cartesian_pose(desired_pose, tolerance, constraints)
      else:
        rospy.logwarn(f"Failed to get transfrom {target_frame} from {reference_frame} ")
        return std_srvs.srv.TriggerResponse(False,"Manipulation-Recheck service routine not completed successfuly, cant obtain transform.") 
    
      if success: #In case of sucessful recheck position -> publish fix tf_frame
          rospy.loginfo("End effector reached the desired pose!")
          rospy.loginfo ("Creating fix")
          fix_target_frame = "fix_cotton_frame_0"  
          fix_reference_frame = target_frame
          manipulation_re_check.broadcast_static_transform(fix_reference_frame,fix_target_frame)
          return std_srvs.srv.TriggerResponse(True,"Manipulation-Recheck service routine completed successfuly.")
      else:
          rospy.logwarn("Failed to reach the desired pose.")
          return std_srvs.srv.TriggerResponse(False,"Manipulation-Recheck service routine not completed successfuly, cant reach target_frame.")

    def handle_net_creator(self,request):
        
      #Populating auxiliar TFs
      aux_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
      total_transforms = []
      #Example variable for demo
      cotton_tf = self.array_actual_moves[0] #Leave this one like this
    
      while not self.tf_listener.canTransform(cotton_tf,"base_link",rospy.Time(0)):
              rospy.loginfo("Waiting for transform base-link -> cotton")
              rospy.sleep(1)

      #Start Counting Time
      if self.start_time == None:
        self.start_time = rospy.Time.now()
      
      self.trials += 1
      
      array_orthodromic_dist = []
      
      #First generate the net using tf utils
      robotbase_to_cotton_transform = self.tf_listener.lookupTransform("base_link",cotton_tf,rospy.Time(0))
      print("Robot Base-Link->Cotton", robotbase_to_cotton_transform)
      #Instance of creation for auxiliar frames
      self.creator_auxiliar_frames = generationNetTFs(robotbase_to_cotton_transform)    
      for subset in range(len(self.z_rotation_angles)): #Interate over vertical axes
          transforms_one_vertical_axis = self.creator_auxiliar_frames.generateTFsOverVerticalAxis(self.z_angles,
                                                                                                  self.z_rotation_angles[subset],
                                                                                                  subset,self.dist_cotton_aux,
                                                                                                  cotton_tf)
          
          point_1 = [transforms_one_vertical_axis[0].transform.translation.x,
                     transforms_one_vertical_axis[0].transform.translation.y,
                     transforms_one_vertical_axis[0].transform.translation.z]
          
          point_2 = [transforms_one_vertical_axis[1].transform.translation.x,
                     transforms_one_vertical_axis[1].transform.translation.y,
                     transforms_one_vertical_axis[1].transform.translation.z]
          
          point_3 = [transforms_one_vertical_axis[2].transform.translation.x,
                     transforms_one_vertical_axis[2].transform.translation.y,
                     transforms_one_vertical_axis[2].transform.translation.z]
          
          array_orthodromic_dist.append(self.__euclidean_distance_vector(self.point_over_heatmap, point_1, self.act_cot_first_translation))
          array_orthodromic_dist.append(self.__euclidean_distance_vector(self.point_over_heatmap, point_2, self.act_cot_first_translation))
          array_orthodromic_dist.append(self.__euclidean_distance_vector(self.point_over_heatmap, point_3, self.act_cot_first_translation))
          
          
          rospy.loginfo("Transforms vertical set: " + str(type(transforms_one_vertical_axis[0].transform.translation)))
          for sub_sub_set in range(len(self.z_angles)): #append one by one the transforms -> format "sendTransform"
              total_transforms.append(transforms_one_vertical_axis[sub_sub_set])
      
      print("DDDDISTANCE")
      print(array_orthodromic_dist)
      heatmap_values = self.__heatmap_function_array(array_orthodromic_dist)
      print(heatmap_values)
      
      aux_static_broadcaster.sendTransform(total_transforms)    
      rospy.sleep(3)

      rospy.loginfo(rospy.get_caller_id() + "First Net creation sucessfully done")
      
      #Calculate heatmap value for each aux_frame 
      
      
      
      manipulation_re_check = CottonMoveItTrajectories(self.tf_listener)

      #Check whether which options have no kinematic solution
      if self.ik_check == True:
        dist = rospy.get_param("/detection_parameters/distance_recheck")
        num_aux_frames = 9 
        list_aux_tfs = []

        for i in range(1, num_aux_frames + 1):
            list_aux_tfs.append(f"aux_frame_{i}")

        publisher_tf_kinematics(dist, list_aux_tfs) #Publish static frames for checker kinematics
        feasibility_list = []

        for i in range(len(list_aux_tfs)):
          frame = f"kin_tf_{i+1}"
          feasibility = manipulation_re_check.ik_feasability(frame)
          feasibility_list.append(feasibility)

        print("Checked kinematic tf returned " +str(feasibility_list) )
      
      #For testing purposes ----- IN THIS PART OF THE CODE THE CANDIDATE VIEW IS SELECTED
      success = manipulation_re_check.is_init_success
      
      #numbers = [1, 2, 3, 4, 6, 7, 8, 9] # Remove central number
      lower_bound_list = 1
      upper_bound_list = 91
      numbers = list(range(lower_bound_list,upper_bound_list))
      print(numbers)
      unvisited = [num for num in numbers if num not in self.list_prev_visited_frames]
      rospy.logwarn("Unvisited nodess:" + str(unvisited))
      random_number = random.choice(unvisited)
      next_move = f"aux_frame_{random_number}"
      
      self.list_prev_visited_frames.append(random_number)
      
      target_frame = next_move
      reference_frame = "base_link"  
 
      transform = manipulation_re_check.get_desired_tf(target_frame, reference_frame)
      #Rewrite transform to put camera_frame
      transform = manipulation_re_check.changeEE_Pose("camera_color_frame",transform)
    
      if transform:
        desired_pose = manipulation_re_check.transform_to_pose(transform)
        tolerance = 0.01 
        constraints = None  
        success = manipulation_re_check.reach_cartesian_pose(desired_pose, tolerance, constraints)
      else:
        rospy.logwarn(f"Failed to get transfrom {target_frame} from {reference_frame} ")
        return std_srvs.srv.TriggerResponse(False,"Manipulation-Recheck service routine not completed successfuly, cant obtain transform.") 
    
      if success and ( next_move == self.array_actual_moves[1] or next_move == self.array_actual_moves[2]): #In case of sucessful recheck position -> publish fix tf_frame
          rospy.loginfo("End effector reached the desired pose!")
          rospy.loginfo ("Creating fix")
          fix_target_frame = "fix_cotton_frame_0"  
          fix_reference_frame = target_frame
          manipulation_re_check.broadcast_static_transform(fix_reference_frame,fix_target_frame)
          return std_srvs.srv.TriggerResponse(True,"Manipulation-Recheck service routine completed successfuly.")
      else:
          rospy.logwarn("Failed to reach the desired pose.")
          return std_srvs.srv.TriggerResponse(False,"Manipulation-Recheck service routine not completed successfuly, cant reach target_frame.")

    def handle_final_approach(self, request):
      
      rospy.loginfo(rospy.get_caller_id() + "Final-Approach motion requested")
      manipulation_final_approach = CottonMoveItTrajectories(self.tf_listener)
    
      success = manipulation_final_approach.is_init_success
      target_frame = "fix_cotton_frame_0"  
      reference_frame = "base_link"  
    
      transform_stamped = manipulation_final_approach.get_desired_tf(target_frame, reference_frame)
    
      #Invert z axis
      trans = [transform_stamped[0][0], transform_stamped[0][1], transform_stamped[0][2]]
      rot = [transform_stamped[1][0], transform_stamped[1][1], transform_stamped[1][2], transform_stamped[1][3] ]

      q_z_180 = tf_trans.quaternion_from_euler(0, 0, 3.14159265)
      
      new_rot = tf_trans.quaternion_multiply(rot,q_z_180)

      baselink_transform_rotatedEE = manipulation_final_approach.TransformMatrixToTransformStamped(target_frame, reference_frame,trans,new_rot)
    

      if baselink_transform_rotatedEE:
        desired_pose = manipulation_final_approach.transform_to_pose(baselink_transform_rotatedEE)
        tolerance = 0.01  
        constraints = None  
    
        success &= manipulation_final_approach.reach_cartesian_pose(desired_pose, tolerance, constraints)
    
        if success and manipulation_final_approach.is_gripper_present:
          success &= manipulation_final_approach.reach_gripper_position(0.85)
          time_elapsed = rospy.Time.now() - self.start_time
          self.csv_writer.writerow([self.experiment_id, self.start_time, time_elapsed.to_sec(), self.trials, self.scenario, str(self.planner), str(self.netconf), str(self.z_angles), str(self.z_rotation_angles), self.rand_latitude_euler, self.rand_longitude_euler])
          rospy.loginfo("Elapsed time: %.2f seconds", time_elapsed.to_sec())
          self.csv_file.flush()

        if success and rospy.get_param('/manipulation_parameters/hardcoded_demo') == "yes":
          success &= hardcoded_pos1()
        elif success and rospy.get_param('/manipulation_parameters/hardcoded_demo') == "no":
          success &= go_to_home_position(self.tf_listener)

        #Create trigger to unlock wait state in perception

        if success:
          rospy.loginfo("End effector reached the desired pose!")
          return std_srvs.srv.TriggerResponse(True,"Manipulation-FinalApproach service routine completed successfuly.")
        else:
          rospy.logwarn("Failed to reach the desired pose.")
          return std_srvs.srv.TriggerResponse(False,"Manipulation-FinalApproach service routine not completed successfuly, cant reach target_frame.")

      else:
        rospy.logwarn(f"Failed to get transfrom {target_frame} from {reference_frame} ")
        return std_srvs.srv.TriggerResponse(False,"Manipulation-FinalApproach service routine not completed successfuly, cant obtain transform.")


if __name__ == '__main__':
    rospy.init_node('simulations_tf_frame_publisher')
    moveit_commander.roscpp_initialize(sys.argv)
    
    #Tf listener
    tf_listener = tf.TransformListener()
    actual_decision_moves = ["fix_act_cotton_1", "aux_frame_3", "aux_frame_2"] #This list contains the real approaches
    
    #csv manage
    csv_path = './real_test1.csv'
    csv_file = open(csv_path, mode='w')
    csv_writer = csv.writer(csv_file)
    
    file_exists = os.path.isfile(csv_path)
    file_empty = os.stat(csv_path).st_size == 0 if file_exists else True #check whether file is empty
    
    if file_empty:
      csv_writer.writerow(["trial_id", "start_timestamp", "duration(s)", "trial_number", "planer", "scenario", "net_conf_1", "separation_x", "separation_y"])
        
    #Generate diffirent cotton tfs
    first_cotton_frame = generalUtilsTF()
    second_cotton_frame = generalUtilsTF()
    fix_act_cotton_frame_1 = generalUtilsTF()
    
    #Broadcaster
    br_static = StaticTransformBroadcaster()
    transforms = []
    
    #Distance from actual_cot to corresponding test
    dis_cot_test = rospy.get_param("/detection_parameters/distance_recheck")
    
    #Creation of some offset displacement-wise
    offset_translation = [-0.15, 0.05, 0]
    #Creation of some offset orientation-wise
    offset_rotation = []
    
    # Static pose for testing
    default_cot_first_translation = [0.55, 0.25, 0.5]

    #Creation of the "test" frames -> Representing initial capture points
    cot_first_translation = [sum(x) for x in zip(offset_translation, default_cot_first_translation)]
    cot_first_rotation_z = [0, 0, -0.7071068, 0.7071068]
    cot_first_rotation_x = [-0.7071068, 0, 0, 0.7071068]
    cot_first_rotation = tf_trans.quaternion_multiply(cot_first_rotation_z,cot_first_rotation_x) 
    
    default_cot_second_translation = [0.65, -0.15, 0.45]
    
    cot_second_translation = [sum(x) for x in zip(offset_translation, default_cot_second_translation)]
    cot_second_rotation_z = [0, 0, -0.7071068, 0.7071068]
    cot_second_rotation_x = [-0.7071068, 0, 0, 0.7071068]
    cot_second_rotation = tf_trans.quaternion_multiply(cot_second_rotation_z,cot_second_rotation_x) 
    
    default_act_cot_first_translation = [default_cot_first_translation[0] + dis_cot_test, default_cot_first_translation[1], default_cot_first_translation[2]]
    
    #Instantiate handle class
    manipulationMovements = handle_states(actual_decision_moves,tf_listener,csv_file, csv_writer, offset_translation, default_act_cot_first_translation )

    fix_act_cot_first_translation = [sum(x) for x in zip(offset_translation, default_act_cot_first_translation)]
    fix_act_cot_first_rotation_z = [0, 0, -0.7071068, 0.7071068]
    fix_act_cot_first_rotation_x = [-0.7071068, 0, 0, 0.7071068]
    fix_act_cot_first_rotation = tf_trans.quaternion_multiply(fix_act_cot_first_rotation_z, fix_act_cot_first_rotation_x)

    
    t_1 = second_cotton_frame.publish_tf_frame(cot_second_translation, cot_second_rotation, parent_frame="base_link", child_frame_name="test_2")
    t_2 = first_cotton_frame.publish_tf_frame(cot_first_translation, cot_first_rotation, parent_frame="base_link", child_frame_name="test_1")
    t_cotto_fix = fix_act_cotton_frame_1.publish_tf_frame(fix_act_cot_first_translation, fix_act_cot_first_rotation, parent_frame="base_link", child_frame_name="fix_act_cotton_1")

    transforms.extend([t_1,t_2,t_cotto_fix])
    
    br_static.sendTransform(transforms)                      
    home_service = rospy.Service('/manipulation/home', std_srvs.srv.Trigger, manipulationMovements.handle_home)
    preApproach_service = rospy.Service('/manipulation/pre_approach', std_srvs.srv.Trigger, manipulationMovements.handle_preapproach)
    first_net_service = rospy.Service('/manipulation/recheck',std_srvs.srv.Trigger, manipulationMovements.handle_net_creator)
    final_approach_service = rospy.Service('/manipulation/final_approach',std_srvs.srv.Trigger, manipulationMovements.handle_final_approach)

    rospy.spin()
    rospy.on_shutdown(shutdown_callback)
    
        