#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
import sys
import numpy as np

class KinovaWaypointController:
    def __init__(self):
        rospy.init_node('kinova_waypoint_control', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander("robot_description")
        print(self.robot.get_group_names())
        self.move_group = moveit_commander.MoveGroupCommander("arm", ns="/my_gen3")
        
        self.move_group.set_planning_time(10)
        self.move_group.allow_replanning(True)
        
        # Define home position
        self.home_position = [0.0, -0.785398, np.pi, -np.pi/2, np.pi, 1.22173, -np.pi/2]
        self.move_group.set_max_velocity_scaling_factor(0.95)
        self.waypoints = [
            [0.10148172236143971, 0.24187992785135926, 1.8415582027335593, -1.2910198796807055, -1.7091784539900523, 1.284471210544881, -2.94132178930618],
            [1.0244433269143842, 0.769469206262273, 1.198158248744297, -1.5418693479360108, -1.3877340720311073, 1.0204909117972338, -2.7115061092284316],
            [0.7883445700317138, 0.7051671143926725, 1.22239918999345, -1.003641122329313, -1.554348451085331, 1.1320926124150725, -2.2803505761723093],
            [0.9073481894316968, -0.7629801051811516, -0.43558761937626267, 1.5789991510633392, -0.7018164900405326, 1.0002598048382607, -1.4270094900519474]
        ]
    
    def move_to_home(self):
        print(self.move_group.get_active_joints())

        rospy.loginfo("Moving to home position")
        self.move_group.set_joint_value_target(self.home_position)
        (success, trajectory, planning_time, error_code) = self.move_group.plan()

        if success:  #In case of planning being sucess
            self.move_group.execute(trajectory, wait=True)
        else:
            rospy.logwarn("Motion planning failed, not executing trajectory.")
        
        rospy.sleep(1.0)
    
    def execute_waypoints(self, repeat_count=2):
        self.move_group.set_goal_joint_tolerance(0.001)
        
        for iteration in range(repeat_count):
            rospy.loginfo(f"Executing waypoint sequence - Iteration {iteration + 1}")
            
            for i, joint_positions in enumerate(self.waypoints):
                rospy.loginfo(f"Moving to Waypoint {i + 1}")
                self.move_group.set_joint_value_target(joint_positions)
                (success, trajectory, planning_time, error_code) = self.move_group.plan()
                    
                if success:  #In case of planning being sucess
                    self.move_group.execute(trajectory, wait=True)
                else:
                    rospy.logwarn("Motion planning failed, not executing trajectory.")
                rospy.sleep(1.0)
            
            rospy.sleep(2.0)
    
    def shutdown(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

def main():
    controller = KinovaWaypointController()
    rospy.wait_for_service('/my_gen3/move_group/plan_execution/set_parameters')
    
    controller.move_to_home()  # Move to home position before executing waypoints
    controller.execute_waypoints(repeat_count=2)
    
    controller.shutdown()

if __name__ == '__main__':
    main()