"""
PreApproach SuperState for DEMETER.
"""

# Generic modules
import rospy

import std_srvs.srv

# State machine
from smach import State
#from utils.ros_communication import MoveItController

# Own modules
from utils.utils_sm import(
    S_CONTINUE, SM_CONTINUE
)

# Manipulation super state (state machine) 
class PreApproach(State):
    """
    State Pre-Approach will contain the server client which will 
    send the required actions to take by the manipulation node
    """

    ## Constructor
    def __init__(self):
        super(PreApproach, self).__init__(outcomes=["exit_preapproach", "return_home"])

    def handle_ready_to_harvest(self,onProcess):
        
        if self.manipulation_in_process == True:
            print("INPROCESS")
        else:
            print("NOTINPROCESS")

    def execute(self,userdata):
        """
        Execution of state machine
        """
        #while self.ableToStart == False:
        #    waypoints = []
        #    start_pose = controller.moveit_client.client.get_move_group().get_current_pose().pose
        #    waypoints.append(start_pose)
        
        #preApproach_service = rospy.Service('/ready_to_harvest', std_srvs.srv.Trigger,handle_ready_to_harvest(userdata.manipulation_in_process))
        
        #Creation client pre-approach manipulation
        rospy.wait_for_service('/manipulation/pre_approach')
        try:
            home_client = rospy.ServiceProxy('/manipulation/pre_approach',std_srvs.srv.Trigger)
            response = home_client()
            rospy.loginfo(response.message)
            
            if response.success == True:
                return "exit_preapproach"
            else:
                print("PROBLEMMM")
                return "return_home"
                
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
