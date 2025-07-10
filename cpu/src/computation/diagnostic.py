#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import traceback

def detailed_moveit_diagnostics():
    try:
        # Initialize ROS node
        rospy.init_node('moveit_detailed_diagnostics', anonymous=True)
        
        # Initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        print("\n--- ROS PARAMETER DIAGNOSTICS ---")
        # Check available parameters
        try:
            params = rospy.get_param_names()
            robot_desc_params = [p for p in params if 'robot_description' in p]
            print("Robot Description Parameters:")
            for param in robot_desc_params:
                print(f"  - {param}")
        except Exception as param_error:
            print(f"Error checking parameters: {param_error}")
        
        print("\n--- NAMESPACE DIAGNOSTICS ---")
        # Try different namespace configurations
        namespaces_to_try = ["/my_gen3", "", None]
        
        for ns in namespaces_to_try:
            print(f"\nTrying Namespace: {ns if ns is not None else 'None'}")
            try:
                # Try RobotCommander
                print("Attempting RobotCommander initialization...")
                robot = moveit_commander.RobotCommander(ns=ns)
                
                # Print group names
                print("Available Move Groups:")
                group_names = robot.get_group_names()
                for group in group_names:
                    print(f"  - {group}")
                
                # Try MoveGroupCommander for each group
                print("\nAttempting MoveGroupCommander for each group:")
                for group in group_names:
                    try:
                        move_group = moveit_commander.MoveGroupCommander(group, ns=ns)
                        print(f"  Successfully initialized group: {group}")
                        
                        # Additional group diagnostics
                        print(f"    - Current Joint Values: {move_group.get_current_joint_values()}")
                        print(f"    - Planning Frame: {move_group.get_planning_frame()}")
                        print(f"    - End Effector Link: {move_group.get_end_effector_link()}")
                    except Exception as group_error:
                        print(f"  Failed to initialize group {group}: {group_error}")
                
            except Exception as robot_error:
                print(f"Failed to initialize RobotCommander: {robot_error}")
                traceback.print_exc()
        
        print("\n--- ADDITIONAL DIAGNOSTICS ---")
        # Check ROS services
        try:
            services = rospy.get_published_topics()
            move_group_services = [s for s in services if 'move_group' in s[0]]
            print("MoveGroup Related Services:")
            for service in move_group_services:
                print(f"  - {service[0]}")
        except Exception as service_error:
            print(f"Error checking services: {service_error}")
    
    except Exception as main_error:
        print(f"Unexpected error: {main_error}")
        traceback.print_exc()
    
    finally:
        # Cleanup
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    detailed_moveit_diagnostics()