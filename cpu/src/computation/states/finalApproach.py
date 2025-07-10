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

from computation.srv import getDetectedInfo
from std_msgs.msg import Time

# Manipulation super state (state machine) 
class FinalApproach(State):
    """
    State Pre-Approach will contain the server client which will 
    send the required actions to take by the manipulation node
    """
    ## Constructor
    def __init__(self):
        super(FinalApproach, self).__init__(outcomes=["exit_final_approach","return_home"],output_keys=['type_of_home_call'])

    def execute(self,userdata):
        """
        Execution of state machine
        """
        n_checks = rospy.get_param('/manipulation_parameters/number_tries_recheck')
        recheck_probability_threshold = rospy.get_param('/manipulation_parameters/recheck_confidence_threshold')
        
        #Before execute the final movement check 
        for i in range(n_checks): #Give it a fix number of tries
            #Retrieve detected info from capture node
            detectedInfo_client = rospy.ServiceProxy('/capture_node/get_info', getDetectedInfo)
            timeObj = Time()
            timeObj.data = rospy.Time.now()
            response_info = detectedInfo_client(timeObj)

            if response_info.bounding_box.bounding_boxes != [] and response_info.bounding_box.bounding_boxes[0].probability >= recheck_probability_threshold:
                #TODO-> Add mechanism to make sure the bounding box selected is the cotton (centered) 
                rospy.loginfo("Retrieved info from capture node not empty [FinalApproach]")
                break
            else: #If boundind box not empty but score to low
                rospy.loginfo("Retrieved info from capture node empty[FinalApproach]")
                rospy.sleep(0.35)
                if i == n_checks-1:
                    rospy.logwarn("Uncomfirmed cotton boll")
                    rospy.logwarn(f"Obtained vector of Bounding boxes:{response_info.bounding_box.bounding_boxes}")
                    #userdata.type_of_home_call = True
                    return "return_home"

        #Creation client Final-approach manipulation
        rospy.wait_for_service('/manipulation/final_approach')
        try:
            finalApproach_client = rospy.ServiceProxy('/manipulation/final_approach',std_srvs.srv.Trigger)
            response = finalApproach_client()
            rospy.loginfo(response.message)
            
            if response.success == True:  
                #userdata.type_of_home_call = True #Variable that handles if the HOME return is to finish or continue harversting
                return "exit_final_approach"
            else:
                #userdata.type_of_home_call = False
                return "return_home"
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        