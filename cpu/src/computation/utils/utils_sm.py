# Global constants


#TODO -> FILE LIKELY TO BE REMOVED, not really clear on the implementation
"""
State end outcomes. These constants describe the possible outcomes when an state ends.

For our case, at most two possibilities can be found: cotton found or cotton not found
"""

# State constants
## State end (single output)
S_CONTINUE = "state_continue"

## State end possibilities (dicotomic output) 
#TODO these in logic for detection and segmentation
S_SUCCESS = "state_success"
S_FAILURE = "state_failure"

# State machine constants
## State Machine end (single output)
SM_CONTINUE = "statemachine_continue"

## State Machine end possibilities (dicotomic output)
SM_SUCCESS = "statemachine_success"
SM_FAILURE = "statemachine_failure"

## DEMETER end state
DEMETER_END = "demeter_end"

#Creation of alternative for easier manage of states

#State Scanning (Scene scanning for cotton detection)
COTTON_FOUND = "cotton_found"
NO_COTTON_FOUND = "no_cotton_found"

#StateMachine Perception related variables
ENTER_PERCEPTION = "Entering Perception State Machine"
EXIT_PERCEPTION = "Leaving Perception State Machine"
PERCEPTION_OUTPUT = "perception_output"


#Detection related states
DETECTION_OUTPUT = "detection_output"
COTTON_DETECTION_END = "Leaving state cotton detection from PERCEPTION"

#Segmentation related states
SEGMENTATION_INPUT = "segmentation_input"
SEGMENTATION_OUTPUT = "segmentation_output"
BOX_SEGMENTATION_END = "Leaving state box segmentation from PERCEPTION"

#StateMachine Localization related variables
ENTER_LOCALIZATION = "Entering Localization State Machine"
EXIT_LOCALIZATION = "Leaving Localization State Machine"

LOCALIZATION_INPUT = "localization_input"
LOCALIZATION_OUTPUT = "localization_output"


#Camera Adjust related states
CAMERA_ADJUST_INPUT = "camera_adjust_input"
CAMERA_ADJUST_END = "Leaving state camera adjustment from LOCALIZATION"
CAMERA_ADJUST_OUTPUT = "camera_adjust_output"
#Cotton Boll capture Depth related states
COTTON_BOLL_DEPTH_END = "Leaving state capturing cotton depth from LOCALIZATION"
COTTON_BOLL_DEPTH_INPUT = "cotton_depth_input"
COTTON_BOLL_DEPTH_OUTPUT = "cotton_depth_output"

#Cotton World TF related states
COTTON_WORLD_COORDINATES_END = "Leaving state obtain cotton world coordinates from LOCALIZATION"
COTTON_WORLD_COORDINATES_INPUT = "cotton_world_coordinates_input"
COTTON_WORLD_COORDINATES_OUTPUT = "cotton_world_coordinates_output"