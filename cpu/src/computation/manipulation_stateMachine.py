#!/usr/bin/env python3
import numpy as np
import rospy
from smach import StateMachine
from sensor_msgs.msg import (
    CameraInfo
)


## DEMETER states
from states.manipulationInit import Init # State
from states.home import Home
from states.wait import WaitManipulation
from states.preapproach import PreApproach   # State 
from states.recheck import Recheck   # State 
from states.finalApproach import FinalApproach 
from states.end import End

## Transitions
from utils.utils_sm import *


def main():
    """
    Auxiliar execution method without inheriting from StateMachine class.
    """
    manipulation_in_process = True
    #type_of_home_call = False  # Set the initial value for type_of_home_call

    # Create container
    sm_top = StateMachine(outcomes=[DEMETER_END])

    sm_top.userdata.manipulation_in_process = manipulation_in_process
    #sm_top.userdata.type_of_home_call = type_of_home_call

    # Add states
    with sm_top:
        # Initialization
        sm_top.add('Init', Init(),
                transitions={SM_CONTINUE:'Home'},
                remapping={'manipulation_in_process':'manipulation_in_process'})
        
        # Home position
        sm_top.add('Home', Home(), #Create new instance for home state
                transitions={'exit_home':'Wait_Manipulation'},)
                #remapping={'type_of_home_call':'type_of_home_call'})
        
        # Wait Manipulation -> This state sends signal to wait_perception to start a new cycle
        sm_top.add('Wait_Manipulation', WaitManipulation(), 
                transitions={'continue_manipulation':'Recheck' , 'finish_manipulation' : 'End_Manipulation'},)
                #remapping={'type_of_home_call':'type_of_home_call'})

        # PreApproach -> Currently unused
        sm_top.add('PreApproach', PreApproach(),
                transitions={'exit_preapproach':'Recheck','return_home':'Home'},
                remapping={'manipulation_in_process':'manipulation_in_process'})
                
        # Recheck
        sm_top.add('Recheck',Recheck(),
                transitions={"exit_recheck":'FinalApproach','return_home':'Home'},
                remapping={'manipulation_in_process':'manipulation_in_process'})

        # FinalApproach
        sm_top.add('FinalApproach',FinalApproach(),
                transitions={"exit_final_approach":'Home', "return_home":"Home"},)
                #remapping={'type_of_home_call':'type_of_home_call'})

        # End_Manipulation -> State when manipulation process finish
        sm_top.add('End_Manipulation',End(),
                transitions={SM_CONTINUE:DEMETER_END})

    # ROS node init
    rospy.init_node('manipulation_state_machine') 

    # Execute state machine
    outcome = sm_top.execute()
    #return outcome

if __name__ == '__main__':
    main()