"""
PreApproach SuperState for DEMETER.
"""

# Generic modules
import rospy

import std_srvs

# State machine
from smach import State

# Own modules
from utils.utils_sm import(
    S_CONTINUE, SM_CONTINUE
)

# Manipulation super state (state machine) 
class Recheck(State):
    """
    State Pre-Approach will contain the server client which will send the required actions to take by the 
    """

    ## Constructor
    def __init__(self):
        super(Recheck, self).__init__(outcomes=["exit_recheck","return_home"])

    # Execution of State
    def execute(self,userdata):
        """
        Execution of state 
        """
        rospy.wait_for_service('/manipulation/recheck')
        try:
            recheck_client = rospy.ServiceProxy('/manipulation/recheck',std_srvs.srv.Trigger)
            response = recheck_client()
            rospy.loginfo(response.message)
            
            if response.success == True:
                return "exit_recheck"
            else:
                return "return_home"
                print("PROBLEMMM")

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)