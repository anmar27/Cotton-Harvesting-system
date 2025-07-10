"""
Selection of cotton SuperState for DEMETER. This class constitutes the state 'Cotton boll selection' of the flow chart.
"""

# Generic modules

# State machine
from smach import State, StateMachine
from smach.user_data import UserData

# Own modules
from ..utils.cotton import Cotton
from ..utils.utils_sm import (
    S_CONTINUE, S_SUCCESS, S_FAILURE,
    SM_CONTINUE, SM_SUCCESS, SM_FAILURE
)

# Ready to harvest cotton?
class RTHCotton(State):

    def __init__(self):
        State.__init__(outcomes = [S_SUCCESS, S_FAILURE], 
                       input_keys = [], 
                       output_keys = [], 
                       io_keys=["sorter"])
        
    def execute(self, ud):
        """
        Execution of state. This state executes:
        
        - Dicotomy to discern wether there's a cotton to collect or not.
        """
        available_rth_cott: bool = ud.sorter.any_rth()
        return S_SUCCESS if available_rth_cott else S_FAILURE

class CriteriaSelection(State):
    """
    Priority selection for DEMETER state machine.
    """

    # Private methods

    # Public methods

    def __init__(self):
        State.__init__(outcomes = [S_CONTINUE],
                         input_keys=["criteria"],
                         output_keys=[],
                         io_keys=["sorter"])

    def execute(self, ud):
        """
        Execution of state. This state performs:

        - Cotton data sorting for best cotton to choose (given by criteria -> lambda function)
        """
        ud.sorter.sort(ud.criteria)    # By score
        return S_CONTINUE


class BollSelection(State):
    """
    Cotton boll to pick selection for DEMETER state machine.
    """

    # Private methods

    # Public methods

    ## Constructor
    def __init__(self):
        State.__init__(outcomes = [S_CONTINUE],
                         input_keys=[],
                         output_keys=["next_cotton"],
                         io_keys=["sorter"])

    def execute(self, ud):
        """
        Execution of state. This state performs:

        - Cotton selection for next cotton boll to pick (first in queue)
        """
        cott: Cotton = ud.sorter.pop()
        ud.next_cotton = cott
        return S_CONTINUE

class CottonSelection(StateMachine):
    """
    Cotton boll selection for DEMETER state machine. Executes everything related to select the cotton to be picked next.
    """

    # Private methods

    # Public methods

    ## Constructor
    def __init__(self):
        """
        CottonSelection state machine takes:
        
        - Input: View class, current view of the camera
        - Output: View class, Results of YOLO prediction (labels, classes, scores)
        """
        StateMachine.__init__(outcomes=[SM_CONTINUE],
                                input_keys=["sorter"],
                                output_keys=["sorter"])

    def execute(self):
        #return super().execute(parent_ud)
        
        # State addition
        self.add("RTHCotton",
                 RTHCotton(),
                 transitions={S_SUCCESS:"CriteriaSelection",
                              S_FAILURE: SM_FAILURE})
        self.add("CriteriaSelection",
                 CriteriaSelection(),
                 transitions={S_CONTINUE:'BollSelection'})    # goto BollSelection
        self.add("BollSelection",
                 BollSelection(),
                 transitions={S_CONTINUE:SM_SUCCESS})   # End to no other state rather than the SM
        
        output = self.execute()
        return output