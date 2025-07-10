# Generic modules
from math import atan, degrees

# ROS
from tf.transformations import (
    quaternion_from_euler, 
)

# State machine
from smach import State

# Own modules
from ..utils.utils_sm import(
    S_CONTINUE
)

class AdjustCameraToBollPosition(State):
    """
    State Init for DEMETER state machine. Initializes everything to start the execution of the robot.
    """

    def __init__(self):
        State.__init__(outcomes=[S_CONTINUE], 
                         input_keys=["centroid_uv"],     # TODO
                         output_keys=[],                 # TODO
                         io_keys=["vision", "roscom"])
        
    def execute(self, ud):
        """
        This state executes a camera adjustment to locate the selected cotton boll in the center of the image

        What does this need?:

        - (io) ROSComunicator (TFs and move the camera move the camera)
        - (io) Vision (we need to compute the movement of the camera to focus the cotton in the center --> TODO create a new function)
        - (input) Cotton centroid = (u,v) computed from image coords
        """

        # Obtain points
        x, y = ud.centroid_uv[0], ud.centroid_uv[1]                         # Points to go
        cx, cy = ud.vision.height//2, ud.vision.width//2                    # Center of image
        fx = ud.vision.K[0][0]                                              # focal length on x
        fy = ud.vision.K[1][1]                                              # Focal length on y

        dx, dy = cx - x, -(cy - y)  # TODO to change, dx and fx are not in the same distance units (viceversa for y)

        # Compute euler angles / traslation from center of scene to desired point
        alpha = degrees(atan(dx / fx))      # Angle of movement from center to point in x coordinate
        beta = degrees(atan(dy / fy))       # Angle of movement from center to point in y coordinate
        q = quaternion_from_euler(alpha, beta, 0)       # TODO Not sure if this is correct

        # Move camera so that centroid_uv == (image.W/2, image.H/2)
        # TODO Hay que tener en cuenta que la rotación es en espacio de cámara, de manera que para rotar el brazo debemos:
        #   newTF: TF_reference -> TF_cam -> add rotation -> TF_reference

        # TODO

        return S_CONTINUE