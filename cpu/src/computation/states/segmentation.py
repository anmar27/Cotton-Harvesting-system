"""
Perception SuperState for DEMETER. This class constitutes the state 'Perception' of the flow chart.
"""
#TODO remove this .py logic already implemented in perception

# Generic modules
import torch
import numpy as np

# State machine
from smach import State, StateMachine
from smach.user_data import UserData

# Own modules
from ..utils.cotton import Cotton
from ..utils.utils_sm import (
    S_CONTINUE,
    SM_CONTINUE
)

# Semantic segmentation state
class SemanticSegmentation(State):

    def __init__(self):
        State.__init__(outcomes = [S_CONTINUE],
                         input_keys=["curr_view", "next_cotton"],         
                         output_keys=["mask"],
                         io_keys=["vision"])
        
    def execute(self, ud):
        """
        Execution of state. This state performs:

        - Segmentation of detected cotton boll
        """
        masks: torch.Tensor = ud.vision.segment(ud.curr_view, ud.next_cotton.label)     # TODO not sure if, given region, returns 1 mask or more
                                                                                        # Let's suppose there can be more than one
        """Tengo que asegurarme de que haya solamente una máscara"""

        # TODO

        ud.mask = masks[0] # Whatever, arreglar esto
        return S_CONTINUE

# Semantic segmentation state
class CentroidComputing(State):

    def __init__(self):
        State.__init__(outcomes = [S_CONTINUE],
                         input_keys=["mask"],         
                         output_keys=["centroid_uv"],
                         io_keys=["vision"])
        
    def execute(self, ud):
        """
        Execution of state. This state performs:

        - Computation of centroid given mask
        """
        ud.centroid_uv = ud.vision.compute_centroid(ud.mask)
        return S_CONTINUE

# Centroid computing super state (state machine) 

class Segmentation(StateMachine):
    """
    State CentroidComputing for DEMETER state machine. Executes everything related to centroid computing
    """

    def __init__(self):
        """
        Perception state machine takes:
        
        - Input: View class, current view of the camera
        - Output: View class, Results of YOLO prediction (labels, classes, scores)
        """
        StateMachine.__init__(outcomes=[SM_CONTINUE],
                                input_keys=["vision", "curr_view", "next_cotton", ],         
                                output_keys=["vision","centroid"])
    
    def execute(self):
        """
        Execution of state machine
        """

        self.add("SemanticSegmentation",
                 SemanticSegmentation(),
                 transitions={
                    S_CONTINUE:"CentroidComputing"})
        self.add("CentroidComputing",
                 CentroidComputing(),
                 transitions={
                    S_CONTINUE:SM_CONTINUE})
        
        outcome = self.execute()
        return outcome