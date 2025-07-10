#!/usr/bin/env python3
import pdb
import rospy
import numpy as np
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf
from geometry_msgs.msg import TransformStamped

#Creation of fix tf called robot_base with respect to rgb_camera_link + Robot_base wrt world

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
        self.robot_base.header.frame_id = 'world'
        self.robot_base.child_frame_id = 'robot_base'
        self.robot_base.transform.translation.x = 0.35
        self.robot_base.transform.translation.y = -0.0
        self.robot_base.transform.translation.z = 0.0 #By the moment assume is static moved down 1 meter
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.robot_base.transform.rotation.x = q[0]
        self.robot_base.transform.rotation.y = q[1]
        self.robot_base.transform.rotation.z = q[2]
        self.robot_base.transform.rotation.w = q[3]

        self.robot_base_camera_base = TransformStamped()
        self.robot_base_camera_base.header.stamp = rospy.Time.now()
        self.robot_base_camera_base.header.frame_id = 'robot_base'
        self.robot_base_camera_base.child_frame_id = 'camera_base'
        self.robot_base_camera_base.transform.translation.x = 0.0
        self.robot_base_camera_base.transform.translation.y = 0.0
        self.robot_base_camera_base.transform.translation.z = 0.45 #By the moment assume is static moved down 1 meter
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.robot_base_camera_base.transform.rotation.x = q[0]
        self.robot_base_camera_base.transform.rotation.y = q[1]
        self.robot_base_camera_base.transform.rotation.z = q[2]
        self.robot_base_camera_base.transform.rotation.w = q[3]

        try:
            self.br.sendTransform([self.robot_base,self.robot_base_camera_base])
            rospy.loginfo('Publishing robot_base frame. Use RViz to visualize')
        except tf.Exception as e:
            rospy.logerr("TF Exception: %s", str(e))
        except rospy.ROSException as e:
            rospy.logerr("ROS Exception: %s", str(e))
        except Exception as e:
            rospy.logerr("Unexpected Exception: %s", str(e))
        
# Main functionality

if __name__ == '__main__':
    
    try:
        rospy.init_node("tf_static_broadcaster_node", anonymous=True)
        ffb = FixedFrameBroadcaster()
        
    except rospy.ROSInterruptException:
        pass
    rospy.spin()