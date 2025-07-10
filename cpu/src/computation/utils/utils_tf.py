"""
Utils in charge of publish and manage all required TFs
"""
import pdb
import math
import copy
import numpy as np

import rospy
import tf
import tf.transformations
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

#TODO likely to be deprecated, can be implemneted by using StaicTransformBroadcaster instead
class FixedFrameBroadcaster():
    """
    Class in charge of broadcasting auxiliar static tf
    """
    _classInstances = 0

    def __init__(self):
        self._classInstances += 1
        self.broadcaster = StaticTransformBroadcaster()
        self.transforms = []
        self.timer = None
    
    def __del__(self):
        self._classInstances -= 1

    def appendTransform(self,parent_frame, child_frame, translation, rotation=(0, 0, 0)):

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        q = tf.transformations.quaternion_from_euler(*rotation) # * for unpacking list elements of "rotation"
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.transforms.append(transform)
        rospy.loginfo(f'Stored transform from {parent_frame} to {child_frame}.')

    def sendAllTransforms(self,event=None):
        if not self.transforms:
            rospy.logwarn("No transform to publish")
        
        try:
            for transform in self.transforms:
                self.broadcaster.sendTransform(transform)
                rospy.loginfo(f'Published transform from {transform.header.frame_id} to {transform.child_frame_id}.')
        except tf.Exception as e:
            rospy.logerr(f"TF Exception: {str(e)}")
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Exception: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Unexpected Exception: {str(e)}")
        
    
    def startBroadcasting(self,rate): # hz
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.sendAllTransforms)

class CottonFrameBroadcaster():
    """
    Class which manages publishing the desired cotton tf
    """
    _classInstances = 0

    def __init__(self,stampedTransform):
        CottonFrameBroadcaster._classInstances += 1

        self.stampedTransform = stampedTransform
        #self.rate  = 10 #hz
        self.broadcaster = StaticTransformBroadcaster()
        #self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.tfPublisherCallback)
        self.is_shutdown = False
        self.publish_static_transform()
    def __del__(self):
        self._classInstances -= 1
    #    self.shutdown() 
    
    #def shutdown(self):
    #    rospy.loginfo("Shutting down TF Publisher")
    #    if self.timer:
    #        self.timer.shutdown()  
    #    self.is_shutdown = True  

    def publish_static_transform(self):
        if self.is_shutdown:
            return
        
        try:
            self.stampedTransform.header.stamp = rospy.Time.now()            
            self.broadcaster.sendTransform(self.stampedTransform)
        except tf.Exception as e:
            rospy.logerr(f"TF Exception: {str(e)}")
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Exception: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Unexpected Exception: {str(e)}")

class generalUtilsTF():
    def __init__(self):
        pass

    def get_desired_tf(self,target_frame,reference_frame):   
        # Initialize TF2 listener
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer)
        while not tf_buffer.can_transform(target_frame,reference_frame,rospy.Time(0),rospy.Duration(1.0)):
          rospy.loginfo(f"Waiting for transform {reference_frame} -> {target_frame}")
        try:
            transform = tf_buffer.lookup_transform(reference_frame, target_frame, rospy.Time(0), rospy.Duration(2.0))
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform lookup failed{e}")
            return None    

    #TODO -> Create parent class with method populate_transform
    def populate_transform(self,trans,total_count,translation,rotation):
        #Creation of tf from base-link to auxiliar_frame 
        trans.header.stamp = rospy.Time.now()

        trans.header.frame_id = "base_link"
        trans.child_frame_id = "aux_frame_"+str(total_count)
        
        trans.transform.translation.x = translation[0]
        trans.transform.translation.y = translation[1]
        trans.transform.translation.z = translation[2]

        trans.transform.rotation.x = rotation[0]
        trans.transform.rotation.y = rotation[1]
        trans.transform.rotation.z = rotation[2]
        trans.transform.rotation.w = rotation[3]

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

    def TransformStampedToTransformMatrix(self,transform_stamped):
      translation = transform_stamped.transform.translation
      rotation = transform_stamped.transform.rotation
      T = tf.transformations.translation_matrix((translation.x,translation.y, translation.z))
      R = tf.transformations.quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
      H = tf.transformations.concatenate_matrices(T,R)

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

class generationNetTFs():
    """
    Class in charge of generating auxiliar static cotton tf
    """

    def __init__(self, robotbase_to_cotton_transform):
        self.robotbase_to_cotton_transform = robotbase_to_cotton_transform
        self.aux_static_broadcaster = StaticTransformBroadcaster()
   
    def populate_transform(self,trans,total_count,translation,rotation):
        #Creation of tf from base-link to auxiliar_frame 
        trans.header.stamp = rospy.Time.now()

        trans.header.frame_id = "base_link"
        trans.child_frame_id = "aux_frame_"+str(total_count)
        
        trans.transform.translation.x = translation[0]
        trans.transform.translation.y = translation[1]
        trans.transform.translation.z = translation[2]

        trans.transform.rotation.x = rotation[0]
        trans.transform.rotation.y = rotation[1]
        trans.transform.rotation.z = rotation[2]
        trans.transform.rotation.w = rotation[3]

    def generateTFsOverVerticalAxis(self, z_vector, angle_z, subset, dist, res_tf):
        transforms = []
        count = 1+(subset*len(z_vector))

        #print(self.robotbase_to_cotton_transform)
        c_xyz = self.robotbase_to_cotton_transform[0]
        c_q = self.robotbase_to_cotton_transform[1]
        c_T = tf.transformations.translation_matrix((c_xyz[0], c_xyz[1], c_xyz[2]))
        c_R = tf.transformations.quaternion_matrix(c_q)

        for angles in range(len(z_vector)):

            #Translation 
            translation_x = -dist*math.sin(angle_z*(math.pi/180))
            translation_y = -dist*math.sin(z_vector[angles]*(math.pi/180))
            translation_z = -dist*math.cos(angle_z*(math.pi/180))
            T = tf.transformations.translation_matrix((translation_x, translation_y, translation_z))
            
            origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
            #Rotation
            roll = -z_vector[angles]*(math.pi / 180.0)
            pitch = angle_z*(math.pi/180.0)
            yaw = 0
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            #q = tf.transformations.quaternion_multiply([0, 1, 0, 0], q)
            Rt = tf.transformations.quaternion_matrix(q)

            baselink_H_auxframe = tf.transformations.concatenate_matrices(c_T, c_R, T, Rt)

            transform = TransformStamped()
            self.populate_transform(transform,
                                    count,
                                    tf.transformations.translation_from_matrix(baselink_H_auxframe),
                                    tf.transformations.quaternion_from_matrix(baselink_H_auxframe))
            transforms.append(transform)
            count += 1 

        print("Sucessfully created TFs over the z axis with a rotation of", angle_z,"radians")
        return transforms