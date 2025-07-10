# State machine
from smach import State
import smach_ros
import rospy

# Own modules
from utils.utils_tf import FixedFrameBroadcaster
from utils.utils_sm import(
    SM_CONTINUE
)

class Init(State):
    """
    State Init for DEMETER state machine. Initializes everything to start the execution of the robot.
    """

    ## Constructor
    def __init__(self):
        # Superclass initialization
        super(Init, self).__init__(outcomes=[SM_CONTINUE])

   

    # Public methods
    
    def execute(self, userdata):
        """
        Execution of state machine
        """
        rospy.loginfo("By the moment no initalization in Manipulation")
        return SM_CONTINUE
        

