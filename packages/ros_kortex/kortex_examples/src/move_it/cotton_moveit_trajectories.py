import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import tf.transformations as tf
import tf2_ros
from math import pi
from std_srvs.srv import Empty
import std_srvs.srv
from geometry_msgs.msg import TransformStamped

#TODO -> Refactor whole code once functional

#Class in charge of creation box boundaries
class boundaries_creation():
  def __init__(self):
    self.robot = moveit_commander.RobotCommander("robot_description")
    self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    self._create_fix_scenario()

  def _box_creator(self,scene,box_ID,box_x,box_y,box_z, pos_x, pos_y, pos_z,rot_z):
        rospy.loginfo("Adding box obstacle to the scene...")
        box_name = f"box_obtacle_{box_ID}"
        box_size = (box_x, box_y, box_z)  # Size of the box (x, y, z)

        # Define the pose of the box
        box_pose = geometry_msgs.msg.Pose()
        box_pose.position.x = pos_x
        box_pose.position.y = pos_y
        box_pose.position.z = pos_z

        roll = 0 
        pitch = 0
        yaw = rot_z  

        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

        box_pose.orientation.x = quaternion[0]
        box_pose.orientation.y = quaternion[1]
        box_pose.orientation.z = quaternion[2]
        box_pose.orientation.w = quaternion[3]

        box_pose_stamped = geometry_msgs.msg.PoseStamped()
        box_pose_stamped.header.frame_id = self.robot.get_planning_frame()
        box_pose_stamped.pose = box_pose

        # Add the box to the scene
        scene.add_box(box_name, box_pose_stamped, size=box_size)
        print(f"Box added id: {box_ID} ")

  def _create_fix_scenario(self):
      """Creates fix scenario of lab setting"""
      scene = self.scene

      self._box_creator(scene,0,1.6,0.1,1.5,-0.55,0.0,0.75,pi/2) #back-plane
      self._box_creator(scene,1,1.1,0.1,1.5,0.1,-0.85,0.75,0) #right-plane
      self._box_creator(scene,2,1.5,1.05,0.05,0.2,0.0,0.025,pi/2) #ground-plane
      self._box_creator(scene,3,0.05,0.05,2,0.6,0.55,1,0) #left bar
      self._box_creator(scene,4,0.05,0.05,2,0.6,-0.6,1,0) #right bar
      self._box_creator(scene,5,1.6,0.1,1.5,0.9,0.0,0.75,pi/2) #front-plane

      rospy.sleep(1) 
  def remove_box(self,box_name):
    """Removes a box from the planning scene"""
    self.scene.remove_world_object(box_name)
    rospy.sleep(2)


class CottonMoveItTrajectories(object):
  """CottonMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(CottonMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cotton_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.arm_group.set_max_velocity_scaling_factor(0.99)
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    arm_group.set_planning_time(10.0)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

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

    arm_group.set_planning_time(15.0) #Redefine timeout

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
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    while not tf_buffer.can_transform(target_frame,reference_frame,rospy.Time(0),rospy.Duration(1.0)):
      rospy.loginfo("Waiting for transform base-link -> aux_frame")

    try:
        # Get the transform from the target_frame to reference_frame
        transform = tf_buffer.lookup_transform(reference_frame, target_frame, rospy.Time(0), rospy.Duration(2.0))
        return transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Transform lookup failed{e}")
        return None
  #TODO -> Implement this methods in utils_tf
  def TransformStampedToTransformMatrix(self,transform_stamped):
    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation
    T = tf.translation_matrix((translation.x,translation.y, translation.z))
    R = tf.quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
    H = tf.concatenate_matrices(T,R)
    
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
    EE_H_cameraFrame = tf.inverse_matrix(cameraFrame_H_EE)
    baseLink_H_cameraFrame = tf.concatenate_matrices(baseLink_H_EE,EE_H_cameraFrame)
    baseLink_T_cameraFrame = tf.translation_from_matrix(baseLink_H_cameraFrame)
    baseLink_R_cameraFrame = tf.quaternion_from_matrix(baseLink_H_cameraFrame)

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


#Handle callback functions

def handle_home(request):

  rospy.loginfo(rospy.get_caller_id() + "Home motion requested")

  #Based if a hardcoded version is desired
  if rospy.get_param('/manipulation_parameters/hardcoded_demo') == 'yes':
    success = hardcoded_pos2()
  else: 
    success = go_to_home_position()

  if not success:
    rospy.logerr("There was an issue when trying to reach home position")
    return std_srvs.srv.TriggerResponse(False,"Problem encountered in Manipulation-Home service routine.")

  return std_srvs.srv.TriggerResponse(True,"Manipulation-Home service routine completed successfuly.")

def handle_preapproach(request):

  rospy.loginfo(rospy.get_caller_id() + "Pre-Approach motion requested")
  manipulation_pre_approach = CottonMoveItTrajectories()

  # For testing purposes
  success = manipulation_pre_approach.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  
  if success:
    rospy.loginfo("Reaching Cartesian Pose...")
    
    actual_pose = manipulation_pre_approach.get_cartesian_pose()
    actual_pose.position.z -= 0.25
    success &= manipulation_pre_approach.reach_cartesian_pose(pose=actual_pose, tolerance=0.001, constraints=None)
  
  if not success:
    rospy.logerr("There was an issue when trying to reach pre-approach position")
    return std_srvs.srv.TriggerResponse(False,"Problem encountered in Manipulation-PreApproach service routine.")

  return std_srvs.srv.TriggerResponse(True,"Manipulation-Home service routine completed successfuly.")

def handle_recheck(request):

  rospy.loginfo(rospy.get_caller_id() + "Re-check motion requested")
  manipulation_re_check = CottonMoveItTrajectories()

  # For testing purposes
  success = manipulation_re_check.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  target_frame = "aux_frame_14"  
  reference_frame = "base_link"  

  """
  for i in range(14):
    target_frame = f"aux_frame_{i+1}"
    transform = manipulation_re_check.get_desired_tf(target_frame, reference_frame)
    transform = manipulation_re_check.changeEE_Pose("camera_color_frame",transform)
    if transform:
      desired_pose = manipulation_re_check.transform_to_pose(transform)
      tolerance = 0.01  # Example tolerance in meters
      constraints = None 
      success &= manipulation_re_check.reach_cartesian_pose(desired_pose, tolerance, constraints)
    else:
      rospy.logwarn(f"Failed to get transfrom {target_frame} from {reference_frame} ")
      return std_srvs.srv.TriggerResponse(False,"Manipulation-Recheck service routine not completed successfuly, cant obtain transform.")
  """    
    #success = manipulation_re_check.reach_cartesian_pose(desired_pose, tolerance, constraints)
  
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

def handle_final_approach(request):

  rospy.loginfo(rospy.get_caller_id() + "Final-Approach motion requested")
  manipulation_final_approach = CottonMoveItTrajectories()

  # For testing purposes
  success = manipulation_final_approach.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  target_frame = "fix_cotton_frame_0"  
  reference_frame = "base_link"  
  
  transform = manipulation_final_approach.get_desired_tf(target_frame, reference_frame)
  
  #Invert z axis
  trans = [transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z]
  rot = [transform.transform.rotation.x,transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w ]

  (roll, pitch, yaw) = tf.euler_from_quaternion(rot)
  new_pitch = pitch + 3.14159
  new_rot = tf.quaternion_from_euler(roll, new_pitch, yaw)
  baselink_transform_rotatedEE = manipulation_final_approach.TransformMatrixToTransformStamped(target_frame, reference_frame,trans,new_rot)
  

  if baselink_transform_rotatedEE:
    desired_pose = manipulation_final_approach.transform_to_pose(baselink_transform_rotatedEE)
    tolerance = 0.01  # Example tolerance in meters
    constraints = None  
  
    success &= manipulation_final_approach.reach_cartesian_pose(desired_pose, tolerance, constraints)
  
    if success and manipulation_final_approach.is_gripper_present:
      success &= manipulation_final_approach.reach_gripper_position(0.85)
    
    if success and rospy.get_param('/manipulation_parameters/hardcoded_demo') == "yes":
      success &= hardcoded_pos1()
    elif success and rospy.get_param('/manipulation_parameters/hardcoded_demo') == "no":
      success &= go_to_home_position()
    
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


def go_to_home_position():
  manipulation_home = CottonMoveItTrajectories()
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

#FIXED FRAMES FOR DEMOS
def hardcoded_pos1():
  manipulation_hardcode1 = CottonMoveItTrajectories()
  arm_group = manipulation_hardcode1.arm_group
  success = True
  tolerance = 0.001
  # Get the current joint positions
  joint_positions = arm_group.get_current_joint_values()
  rospy.loginfo("Printing current joint positions before movement :")
  for p in joint_positions: rospy.loginfo(p)
  # Set the goal joint tolerance
  manipulation_hardcode1.arm_group.set_goal_joint_tolerance(tolerance)
  
  # Set the joint target configuration
  if manipulation_hardcode1.degrees_of_freedom == 7:
    joint_positions[0] = 0 
    joint_positions[1] = -0.785398
    joint_positions[2] = pi
    joint_positions[3] = -1.48353
    joint_positions[4] = pi
    joint_positions[5] = 0.785398
    joint_positions[6] = -pi/2

  arm_group.set_joint_value_target(joint_positions)
  
  # Plan and execute in one command
  success &= arm_group.go(wait=True)

  if success and manipulation_hardcode1.is_gripper_present:
      success &= manipulation_hardcode1.reach_gripper_position(0)
  # Show joint positions after movement
  new_joint_positions = arm_group.get_current_joint_values()
  rospy.loginfo("Printing current joint positions after movement :")
  for p in new_joint_positions: rospy.loginfo(p)
  return success
def hardcoded_pos2():
  manipulation_hardcode2 = CottonMoveItTrajectories()
  arm_group = manipulation_hardcode2.arm_group
  success = True
  tolerance = 0.001
  # Get the current joint positions
  joint_positions = arm_group.get_current_joint_values()
  rospy.loginfo("Printing current joint positions before movement :")
  for p in joint_positions: rospy.loginfo(p)
  # Set the goal joint tolerance
  manipulation_hardcode2.arm_group.set_goal_joint_tolerance(tolerance)
  
  # Set the joint target configuration
  if manipulation_hardcode2.degrees_of_freedom == 7:
    joint_positions[0] = 0.03874631
    joint_positions[1] = -1.0164798
    joint_positions[2] = pi
    joint_positions[3] = -2.29231544
    joint_positions[4] = pi
    joint_positions[5] = 0.42184608
    joint_positions[6] = -pi/2

  arm_group.set_joint_value_target(joint_positions)
  
  # Plan and execute in one command
  success &= arm_group.go(wait=True)

  if success and manipulation_hardcode2.is_gripper_present:
      success &= manipulation_hardcode2.reach_gripper_position(0)
  # Show joint positions after movement
  new_joint_positions = arm_group.get_current_joint_values()
  rospy.loginfo("Printing current joint positions after movement :")
  for p in new_joint_positions: rospy.loginfo(p)
  return success


def hardcoded_home():
  manipulation_hhome = CottonMoveItTrajectories()
  arm_group = manipulation_hhome.arm_group
  success = True

  rospy.loginfo(rospy.get_caller_id() + "Home motion requested")
  #success = go_to_home_position()
  #FOR DEMO PURPOSES
  success = hardcoded_pos1()
  success &= hardcoded_pos2()

  if success and manipulation_hhome.is_gripper_present:
      success &= manipulation_hhome.reach_gripper_position(0)

  if not success:
    rospy.logerr("There was an issue when trying to reach home position")
    return std_srvs.srv.TriggerResponse(False,"Problem encountered in Manipulation-Home service routine.")

  return std_srvs.srv.TriggerResponse(True,"Manipulation-Home service routine completed successfuly.")


def shutdown_callback():
  if rospy.get_param('/manipulation_parameters/hardcoded_demo') == 'yes':
    success = hardcoded_pos1()
  else:
    success = go_to_home_position()

  if success:
    rospy.loginfo("Sucessfully returned to home position prior shutdown")
  else:
    rospy.loginfo(success)
    rospy.logerr("There was a problem when going to home position prior shutdown")

class publisherFrame:
  def __init__(self):
    self.br = tf2_ros.TransformBroadcaster()
  def broadcast_rotated_baselink(self,event):
    t = geometry_msgs.msg.TransformStamped()
    # Set the header information
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "new_base_link"

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0

    q1 = tf.quaternion_from_euler(0,pi/2,0)
    q2 = tf.quaternion_from_euler(0,0,-pi/2)
    q = tf.quaternion_multiply(q1,q2)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # Send the transform
    self.br.sendTransform(t)


def main():
  """
  ROS-NODE that handles request from manipulation state machines
  """
  publish_frame = publisherFrame()
  rospy.init_node('cotton_move_it_trajectories')
  created_scenario = boundaries_creation()
  rospy.Timer(rospy.Duration(0.1),publish_frame.broadcast_rotated_baselink)

  #Create var to handle hardcoded demo 
  if rospy.get_param('/manipulation_parameters/hardcoded_demo') == "yes":
    is_hardcoded_demo = True
    hardcoded_home()
  else:
    is_hardcoded_demo = False

  #Creation of manipulation services
  home_service = rospy.Service('/manipulation/home', std_srvs.srv.Trigger, handle_home)
  preApproach_service = rospy.Service('/manipulation/pre_approach', std_srvs.srv.Trigger,handle_preapproach)
  recheck_service = rospy.Service('/manipulation/recheck',std_srvs.srv.Trigger,handle_recheck)
  final_approach_service = rospy.Service('/manipulation/final_approach',std_srvs.srv.Trigger,handle_final_approach)
  
  rospy.spin()
  rospy.on_shutdown(shutdown_callback)

if __name__ == '__main__':
  main()
