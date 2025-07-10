# State machine
from smach import State
import rospy
import std_srvs.srv

# Own modules
from utils.utils_sm import(
    S_CONTINUE,
    SM_CONTINUE
)

class Wait(State):
    """
    Base Wait state for DEMETER state machines.
    """
    def __init__(self, outcomes):
        State.__init__(self, outcomes=outcomes)
        self.flag_waiting = True
        self.continue_execution = False
        
    def handle_ready(self, request):
        """
        Base handler for ready service callbacks.
        """
        if request.data:
            self.flag_waiting = False
            self.continue_execution = True
            return std_srvs.srv.SetBoolResponse(True, "Service callback handled successfully.")
        else:
            self.continue_execution = False
            return std_srvs.srv.SetBoolResponse(False, "Service not handled successfully")
            
    def signal_other_state(self, service_name):
        """
        Helper method to signal the other state machine
        """
        try:
            rospy.wait_for_service(service_name)
            signal_service = rospy.ServiceProxy(service_name, std_srvs.srv.SetBool)
            resp = signal_service(True)
            if resp.success:
                rospy.loginfo(f"Successfully signaled {service_name}")
                rospy.loginfo(resp.message)
                return True
            else:
                rospy.logerr(f"Problem found in signaling {service_name}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def execute(self, ud):
        """
        Base execution method to be overridden by child classes.
        """
        raise NotImplementedError("Execute method must be implemented by child classes")


class WaitPerception(Wait):
    """
    Wait state for DEMETER perception state machine.
    """
    def __init__(self):
        super().__init__(outcomes=['continue_perception', 'finish_perception'])
        # Creation service server to return with the perception -> Triggered from worldTf state
        self.manipulation_ready_server = rospy.Service(
            '/perception_ready_to_start',
            std_srvs.srv.SetBool,
            self.handle_ready_to_perceive
        )
        
    def handle_ready_to_perceive(self, request):
        """
        Handler for perception-specific ready service.
        """
        return self.handle_ready(request)
        
    def execute(self, ud):
        rospy.loginfo("Entering Wait Perception State")
        self.flag_waiting = True
        
        # Signal manipulation state machine every time
        if not self.signal_other_state('/manipulation_ready_to_start'):
            return 'finish_perception'
            
        while self.flag_waiting:
            print("Waiting to receive Trigger from manipulation SM")
            rospy.sleep(1)
            
        if self.continue_execution:
            return 'continue_perception'
        else:
            return 'finish_perception'


class WaitManipulation(Wait):
    """
    Wait state for DEMETER manipulation state machine.
    """
    def __init__(self):
        super().__init__(outcomes=['continue_manipulation', 'finish_manipulation'])
        # Creation service server for manipulation
        self.perception_ready_server = rospy.Service(
            '/manipulation_ready_to_start',
            std_srvs.srv.SetBool,
            self.handle_ready_to_manipulate
        )
        self.first_execution = True
        
    def handle_ready_to_manipulate(self, request):
        """
        Handler for manipulation-specific ready service.
        """
        return self.handle_ready(request)
        
    def execute(self, ud):
        rospy.loginfo("Entering Wait Manipulation State")
        self.flag_waiting = True
        
        # Signal perception state machine except for the first time
        if not self.first_execution:
            if not self.signal_other_state('/perception_ready_to_start'):
                return 'finish_manipulation'
        else:
            self.first_execution = False
            
        while self.flag_waiting:
            print("Waiting to receive Trigger from perception SM")
            rospy.sleep(1)
            
        if self.continue_execution:
            return 'continue_manipulation'
        else:
            return 'finish_manipulation'