"""
Perception SuperState for DEMETER. This class constitutes the state 'Perception' of the flow chart.
"""

# Generic modules
import rospy
# State machine
from smach import State, StateMachine
from utils.vision import Detection, Segmentation

# Own modules
from utils.utils_sm import *

# Cotton detection state
class CottonDetection(State):

    def __init__(self):
        """
        State.__init__(self,outcomes = [S_CONTINUE],
                         input_keys=["curr_view"],         
                         output_keys=["cotton_detection"],
                         io_keys=["vision", "sorter"])
        """
        State.__init__(self,outcomes = [COTTON_DETECTION_END],
                        output_keys=[DETECTION_OUTPUT])
        #self.visionDetection = Detection()
        
    def execute(self, userdata):
        """
        Execution of state. This state performs:

        - Cotton detection and classification with YOLO
        """
        rospy.loginfo("Entering into Cotton Detection state")
        
        rospy.sleep(0.5)
        """
        # Loop until detection info is available
        while True:
            #with self.visionDetection.lock:  # Ensure thread-safe access
            detectionInfo = self.visionDetection.getDetectedInfo()
            print("CHECKINFO")
            print(detectionInfo)
            if None in detectionInfo or any(item == [] for item in (detectionInfo)):
                rospy.loginfo("Not Detected Info yet!!")
            else:
                break

        #rospy.sleep(10)
        print("INFO INSIDE EXECUTE")
        print(detectionInfo)
        rospy.loginfo("Cotton Detection state execution completed.")
        rospy.loginfo("Passing info to Segmentation Module.")
        userdata.detection_output = detectionInfo
        #preds: dict = ud.vision.predict(ud.curr_view)
        #ud.sorter.update(preds)
        """
        #TODO introduce logic in case of negative result
        return COTTON_DETECTION_END

class BoxSegmentation(State):
    def __init__(self):
        State.__init__(self,outcomes=[BOX_SEGMENTATION_END],
                       #input_keys=[SEGMENTATION_INPUT],
                       output_keys=['centroid2D', 'point_cloud'],)

        self.visionSegmentation = Segmentation()

    def execute(self, userdata):
        """
        Execution of state. This state performs:
        -Segment cotton based on passed box params
        """
        #detectedPassedInfo = userdata.segmentation_input
        rospy.loginfo("Entering into Cotton Segmentation state")
        #rospy.loginfo(detectedPassedInfo)

        success = False
        #TODO redirect statemachine to take a new detected image 
        while not success:
            successful_segmentation = self.visionSegmentation.serviceRequestSegmentation("detectedPassedInfo")
            if successful_segmentation == True:
                success = True
            else:
                rospy.logwarn("Segmentation Failed, Retrying...") 
        
        userdata.centroid2D = self.visionSegmentation.getCentroidValue2DSpace()
        userdata.point_cloud = self.visionSegmentation.getPointCloud()
        return BOX_SEGMENTATION_END

# Perception super state (state machine) 
class Perception(StateMachine):
    """
    State Perception for DEMETER state machine. Executes everything related to cotton detection.
    """
    ## Constructor
    def __init__(self):
        """
        Perception state machine takes:
        
        - Input: View class, sorter class, current view of the camera
        - Output: View class, sorter clas, results of YOLO prediction (labels, classes, scores)
        """
        #Creation of Sub StateMachine Perception
        '''
        StateMachine(self,outcomes=[S_CONTINUE],
                                input_keys=["vision", "sorter", "curr_view"],         
                                output_keys=["vision", "sorter", "cotton_detection"])
        '''
        
        StateMachine.__init__(self, outcomes=[EXIT_PERCEPTION],
                              output_keys = ['centroid2D','point_cloud'])
        with self:
            
            StateMachine.add('BoxSegmentation',BoxSegmentation(),
                            transitions={BOX_SEGMENTATION_END:EXIT_PERCEPTION},
                            remapping={'centroid2D': 'centroid2D',
                                       'point_cloud':'point_cloud'})
    # Private methods

    # Public methods
        
    # Execution of State
    def execute(self,userdata):
        """
        Execution of state machine
        """
        # Execute stateand return exit status (success or failure)
        rospy.loginfo("Executing Perception state machine")
        outcome = super(Perception, self).execute(userdata)
        return outcome