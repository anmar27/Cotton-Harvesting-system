import numpy as np
from math import cos, sin, floor
import cv2 as cv

#TODO likely to  remove file, alredy implemented in ROS default install
# Geometry uutils

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    Create a rotation matriz from a given quaternion.

    Extracted from: https://github.com/joycesudi/quaternion/blob/main/quaternion.py
    
    if a quaternion is [qx qy qz qw], with real part qw, the correspondence is:
    
    qx = qx, qy = qy, qz = qz, qw = qw
    """
    # Create the rotation matrix from the quaternion
    rotation_matrix = np.array([[qw*qw + qx*qx - qy*qy - qz*qz, 2*qx*qy - 2*qw*qz, 2*qw*qy + 2*qx*qz],
                                [2*qw*qz + 2*qx*qy, qw*qw - qx*qx + qy*qy - qz*qz, 2*qy*qz - 2*qw*qx],
                                [2*qx*qz - 2*qw*qy, 2*qw*qx + 2*qy*qz, qw*qw - qx*qx - qy*qy + qz*qz]])
    return rotation_matrix

def rotationMatrixToQuaternion(R:np.ndarray) -> np.ndarray:
    """
    Creates a quaternion from a rotation matrix defining a given orientation. 

    Extracted from: https://github.com/joycesudi/quaternion/blob/main/quaternion.py
    """    
    u_q0 = np.sqrt((1 + R[0,0] + R[1,1] + R[2,2])/4) # the prefix u_ means unsigned
    u_q1 = np.sqrt((1 + R[0,0] - R[1,1] - R[2,2])/4)
    u_q2 = np.sqrt((1 - R[0,0] + R[1,1] - R[2,2])/4)
    u_q3 = np.sqrt((1 - R[0,0] - R[1,1] + R[2,2])/4)
    
    q = np.array([u_q0, u_q1, u_q2, u_q3])
    
    if u_q0 == max(q):
        q0 = u_q0
        q1 = (R[2,1] - R[1,2])/(4*q0)
        q2 = (R[0,2] - R[2,0])/(4*q0)
        q3 = (R[1,0] - R[0,1])/(4*q0)
        
    if u_q1 == max(q):
        q1 = u_q1
        q0 = (R[2,1] - R[1,2])/(4*q1)
        q2 = (R[0,1] + R[1,0])/(4*q1)
        q3 = (R[0,2] + R[2,0])/(4*q1)
    
    if u_q2 == max(q):
        q2 = u_q2
        q0 = (R[0,2] - R[2,0])/(4*q2)
        q1 = (R[0,1] + R[1,0])/(4*q2)
        q3 = (R[1,2] + R[2,1])/(4*q2)    
        
    if u_q3 == max(q):
        q3 = u_q3
        q0 = (R[1,0] - R[0,1])/(4*q3)  
        q1 = (R[0,2] + R[2,0])/(4*q3)
        q2 = (R[1,2] + R[2,1])/(4*q3)  
      
    q = np.array([q1, q2, q3, q0])   
    return q

# Rotations

def rotRx(rad_angle):
    Rx = np.identity(3)
    Rx = np.array([ 
        [1.0, 0.0,0.0],
        [0.0, cos(rad_angle), -sin(rad_angle)],
        [0.0, sin(rad_angle),  cos(rad_angle)]
    ])
    return Rx

def rotRy(rad_angle):
    Ry = np.identity(3)
    Ry = np.array([
        [cos(rad_angle), 0.0, sin(rad_angle)],
        [0.0,1.0,0.0],
        [-sin(rad_angle), 0.0, cos(rad_angle)]
    ])
    return Ry

def rotRz(rad_angle):
    Rz = np.identity(3)
    Rz = np.array([ 
        [cos(rad_angle), -sin(rad_angle),0.0],
        [sin(rad_angle),  cos(rad_angle), 0.0],
        [0.0,0.0 ,  1.0]
    ])
    return Rz

# Homography related methods

def translate(x:float, y:float, z:float) -> np.ndarray:
    """
    Construct a translation column vector (3x1) from coordinates.
    """
    return np.array([x,y,z]).T

def buildHomogeneous(R: np.ndarray, t:np.ndarray):
    """
    Returns the homogeneous transform given by the rotation and transformation given
    """
    H_a2b = np.identity(4)
    H_a2b[:3,:3] = R
    H_a2b[:3, 3] = t.T
    return H_a2b

def rotationTranslation(H: np.ndarray) -> tuple:
    """
    Retrieve the rotation and translation from an homogeneous transformation.
    """
    rot = H[:3, :3]
    trans = H[:3, 3]
    return (rot, trans)

# 2D point to 3D

def unproject_2Dto3D(point: np.ndarray, d: float,
                    K: np.ndarray, dist: np.ndarray, 
                    H_cam2world: np.ndarray) -> np.ndarray:
    """
    Transforms 2D point in camera space to world space.

    point = (u,v), depth in meters

    TODO asegurarse de que dist_coeffs = np.array([k1, k2, p1, p2, k3]), en este orden
    """

    undistorted_point = cv.undistortPoints(point, K, dist, P=K)
    u_undistorted, v_undistorted = undistorted_point[0][0]

    # Unproject to 3D camera coordinates
    x, y = (u_undistorted - K[0][2]) / K[0][0], (v_undistorted - K[1][2]) / K[1][1]
    camera_coords = np.array([x * d, y * d, d, 1])
    # Transform to world coordinates
    world_coords = H_cam2world @ camera_coords

    # The resulting world coordinates
    world_coords = world_coords.flatten()   # TODO is it really necessary?
    return world_coords