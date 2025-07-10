"""
HOME SuperState for DEMETER.
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
class Home(State):
    """
    State Home will contain the server client which will 
    send the required actions to take by the manipualtion node
    """

    ## Constructor
    def __init__(self):
        super(Home, self).__init__(outcomes=["exit_home"],input_keys=['type_of_home_call'])
        print("Home position constructor called")

    def execute(self,userdata):
        #Creation client Home manipulation
        rospy.wait_for_service('/manipulation/home')
        try:
            home_client = rospy.ServiceProxy('/manipulation/home',std_srvs.srv.Trigger)
            response = home_client()
            rospy.loginfo(response.message)
            
            if response.success == True:
                return "exit_home"
            else:
                print("PROBLEMMM")
                #TODO -> Define routine if no solution found to came back home 

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
