#!/usr/bin/env python3
import numpy as np
import rospy
from smach import StateMachine
from sensor_msgs.msg import (
    CameraInfo
)

## DEMETER states
from states.initialization import Init          # State
from states.perception import Perception        # State machine
#from states.selection import CottonSelection    # State machine
#from states.segmentation import Segmentation    # State machine
from states.localization import Localization    # State machine
#from states.manipulation import Manipulation    # State machine
from states.wait import WaitPerception
from states.end import End                      # State

## Transitions
from utils.utils_sm import *

def main():
    """
    Auxiliar execution method without inheriting from StateMachine class.
    """
    # Create container
    sm_top = StateMachine(outcomes=[DEMETER_END])

    # Add states
    with sm_top:
        # Initialization
        sm_top.add('Init', Init(),
                transitions={SM_CONTINUE:'Perception'})
        
        # Perception
        
        #TODO implement logic on StateMachine to depending on detected data: go for segmentation or move base robot
        sm_top.add('Perception', Perception(),
                transitions={EXIT_PERCEPTION:'Localization'},
                remapping={'centroid2D':'centroid2D',
                           'point_cloud':'point_cloud'})
        
        # Localization
        sm_top.add('Localization',Localization(),
                transitions={EXIT_LOCALIZATION:'End',
                             "continue_perception":'Wait_Perception'},
                remapping={'centroid2D': 'centroid2D',
                           'point_cloud':'point_cloud',
                            LOCALIZATION_OUTPUT: 'centroid_world_tf'})

        sm_top.add('Wait_Perception',WaitPerception(),
                transitions={'continue_perception':'Perception',
                             "finish_perception":'End'},)
                #remapping={LOCALIZATION_INPUT: 'centroid_image',
                #           LOCALIZATION_OUTPUT: 'centroid_world_tf'})
        '''
        # Cotton boll selection
        sm.add('Selection',
                CottonSelection(),
                transitions={SM_SUCCESS:'Segmentation',
                            SM_FAILURE:'End'})
        # Segmentation
        sm.add('Segmentation',
                Segmentation(),
                transitions={SM_CONTINUE:'Localization'})
        
        # Manipulation
        sm.add('Manipulation',
                Manipulation(),
                transitions={SM_CONTINUE:'Selection'})
        '''
        # End
        sm_top.add('End',
                End(),
                transitions={SM_CONTINUE:DEMETER_END})   # End the execution
    
    # ROS node init
    #rospy.init_node(sm.userdata.config.get('node_name'))
    rospy.init_node('perception_state_machine') 

    # Execute state machine
    outcome = sm_top.execute()
    #return outcome

if __name__ == '__main__':
    main()