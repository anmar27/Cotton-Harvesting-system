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
        self.transform_broadcaster = FixedFrameBroadcaster()

        #Configure resolution based on camera in use
        if rospy.get_param('/detection_parameters/camera_used') == 'kinova':
            rospy.set_param('/detection_parameters/resolution_camera', {'width': 1280, 'height': 720})
        else:
            rospy.set_param('/detection_parameters/resolution_camera', {'width': 2048, 'height': 1536})
    # Public methods
    
    def execute(self, userdata):
        """
        Execution of state machine
        """
        rospy.loginfo("Entering to Initalization state")
        rospy.loginfo("Initalization all the TFs")
        rospy.loginfo("Init state execution completed.")

        return SM_CONTINUE
        

