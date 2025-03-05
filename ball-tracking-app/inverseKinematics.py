

import numpy as np
import matplotlib.pyplot as plt
from typing import Any


def rotation_matrix(roll: float, pitch: float) -> np.ndarray:
    """
    Computes the combined rotation matrix for a given roll and pitch angle.

    This function creates a 3x3 rotation matrix that first applies a rotation 
    around the X-axis by the given roll angle, and then applies a rotation 
    around the Y-axis by the given pitch angle. The angles are assumed to be in degrees.

    Parameters:
    roll (float): The roll angle in degrees (rotation around the X-axis).
    pitch (float): The pitch angle in degrees (rotation around the Y-axis).

    Returns:
    np.ndarray: The 3x3 rotation matrix resulting from the combined roll and pitch rotations.
    """

    roll = np.radians(roll)
    pitch = np.radians(pitch)
    
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    return R_x @ R_y


def inverse_kinematics(roll: float, pitch: float, height: float) -> np.ndarray:
    """
    Calculates the primary joint angles for a 3RRS platform robot.

    Args:
        roll (float): The roll angle in radians.
        pitch (float): The pitch angle in radians.
        height (float): The height value of the platform in [mm].

    Returns:
        servo angles (np.ndarray): Servo angles in degrees
    """
    
    a = np.array([
        [0, 50, 0],
        [50 * np.sin(np.radians(60)), -50 * np.cos(np.radians(60)), 0],
        [-50 * np.sin(np.radians(60)), -50 * np.cos(np.radians(60)), 0]
    ])
    
    b = a.copy()
    R = rotation_matrix(roll, pitch)
    P = np.array([0, 0, height])
    transformed_b = (R @ b.T).T + P
    
    S_mat_norm = np.linalg.norm(transformed_b - a, axis=1)

    servo_angles = np.degrees(np.arccos((S_mat_norm**2 - 2*35**2)/(-2*35**2))/2)

    return servo_angles