

import numpy as np
import matplotlib.pyplot as plt
import typing

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