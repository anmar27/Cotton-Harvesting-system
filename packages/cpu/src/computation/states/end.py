# State machine
from smach import State
import rospy

# Own modules
from utils.utils_sm import(
    S_CONTINUE,
    SM_CONTINUE
)

class End(State):
    """
    End state for DEMETER state machine.
    """

    ## Constructor
    def __init__(self):
        State.__init__(self, outcomes = [SM_CONTINUE])
        
    def execute(self, ud):
        """
        Execution of state. This state performs:
        
        - [Saving of current configuration into config.json file]
        - TODO more to come
        """
        
        rospy.loginfo("Entering End State")
        # Save current config state --> Is is really necessary?
        rospy.sleep(1)
        # Turn down everything
        rospy.loginfo("End state completed.")

        return SM_CONTINUE