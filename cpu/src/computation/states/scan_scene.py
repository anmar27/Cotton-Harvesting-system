# Overall description
"""
State ScanScene. This state scans the entire scene looking for cotton bolls, so that the obtained bolls are truly RTH cotton bolls.

The workflow will go as follows:

    - The robot will move and detect cotton bolls. When a cotton boll is detected, the data about its location is saved.
    - The robot continues. If the cotton is seen for a number of frames (5,6,7...)
"""

# How we do tracking knowing we have to detect the same cotton boll?
"""
    1) By computing the centroid we get the coordinates {x,y,z} with a given estimation error. When moving the arm, we know how far the arm
        has moved, so we can check the cotton is in the same position, and thus comparations can be done
    2) Everytime the arm moves and the same cotton is detected, we can consider the detected cottons as new. When {x,y,z} are obtained, 
        compute euclidean distance to already detected cotton bolls and relate the new one to the nearest, if distance between them
        is less than some error e
"""

# libraries
import rospy
import ultralytics
from smach import State

from utils.utils_sm import COTTON_FOUND, NO_COTTON_FOUND

# Scanning state

class Scanning(State):

    # Private methods

    def __build_kinova_path(self) -> list:
        """
        Function that builds a kinova path to follow.

        The returning value is a list of Transforms for the robot to follow
        """
        steps: list = []

        # Streps construction (TODO)

        return steps
    
    def __move_kinova(self, step) -> list:
        """
        Send a transform to the ROS node Kinova to move the robotic arm.

        step: Transform that describes the translation and rotation for the next step of the movement
        """

    # Public methods

    def __init__(self, outcomes=[], input_keys=[], output_keys=[], io_keys=[]): # To be modified
        super().__init__(outcomes, input_keys, output_keys, io_keys)

        """
        In the userdata attribute we have:
            - yolo: YOLO model 
            - kinova: commmunication with the robotic arm
        """
        # Class data
        self.frames_end: int = 5    # Number of frames to end the execution   

    def execute(self, ud):
        """
        Execute the scanning of the scene.

        The execution of the state ends when a detection is persistent through some frames
        """

        # PSEUDOCODE:
        """
        int frames_detected = 0
        list steps = obtain_kinova_path()   # List of transforms that the kinova arm is going to take
                                            # At the moment we don't know the shape of the path

        for i in range(len(steps)):
            move_kinova(streps[i])
            image = get_current_vision()
            res = detect_yolo(image)

            if res is not None:
                frames_detected += 1
            else:
                frames_detected = 0

            if frames_detected >= frames_end:
                return COTTON_FOUND         # End the state machine execution with positive result

        return NO_COTTON_FOUND              # End the state machine execution with negative result
        """

        # CODE
        frames_detected: int = 0
        steps: list = self.__build_kinova_path()

        for i in range(len(steps)):
            self.__move_kinova(steps[i])     # Move the kinova

            
            if res is not None:
                frames_detected += 1
            else:
                frames_detected = 0

            if frames_detected >= self.frames_end:
                return COTTON_FOUND         # End the state machine execution with positive result

        return NO_COTTON_FOUND              # End the state machine execution with negative result

# main