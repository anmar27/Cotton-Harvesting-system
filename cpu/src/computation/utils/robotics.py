"""
Robotics manipulator. This file implements a manipulation class to communicate with the pieces of the robotic implementation
(UR5, gripper, and maybe more).

TODO things:

    - Not sure at the moment what robotic arm we'll use in this project.
    - TFs will be here or in ros_com? Discuss it with Antonio

    - Maybe change the implementation of all robotics in one class to one class per robot part?
"""

# Generic modules
from urx import Robot

# ROS modules
import rospy
## Transforms
import tf
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Own modules

# Robot class
class Robotics(object):
    """
    Communicate with the robotic parts of the DEMETER implementation for movement, calibration, etc.
    """

    def __init__(self, ur_ip: str) -> None:
        """Constructor of class"""

        self.ur5: Robot = Robot(host=ur_ip) # IP of the robotic arm (UR5 at the moment)
        self.gripp = None                   # Grippper of the robotic arm (To be discussed)

        pass

    # TODO

    # Termination

    def end(self):
        """
        Terminate all processes of the robot and leave the hardware in an standby state.
        """

        self.ur5.close()    # UR5 closing

        return
