

import cv2 as cv
import numpy as np
from typing import Type, Tuple


def nothing(x):pass


def track(capture:Type[cv.VideoCapture]) -> Tuple[bool, np.ndarray]:
    """
    Capture camera data, process image, and detect position of the ball.

    Args:
        capture (cv.VideoCapture): The frame capture object from the USB camera.

    Returns:
        Tuple[bool, np.ndarray]: A tuple containing:
            - bool: capture return status (True if frame was successfully read)
            - np.ndarray: The chosen circle's coordinates (x, y) and radius in the format [x, y, radius]
    """

    chosen = np.array([0,0,0], np.int32)
    prevCircle = chosen

    dist = lambda x1,y1,x2,y2: (x1-x2)**2+(y1-y2)**2

    ret, frame = capture.read()
    
    # Crop image to fit around platfrom plate
    frame = frame[240-200:240+200, 320-200:320+200]

    # Flip around y-axis to match platfrom coordinate frame
    frame = cv.flip(src=frame, flipCode=1)

    # Applying a Gaussian Blur to the grayscale image
    vid_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #gaussian_blur = cv.GaussianBlur(vid_gray, (17, 17), 0)

    # Apply circle mask
    mask = np.zeros_like(vid_gray)
    mask = cv.circle(img=mask, center=(200,200), radius=190, color=(255,255,255), thickness=-1)

    masked_image = cv.bitwise_and(mask, vid_gray)

    # Detect Circle using HoughCircles
    circles = cv.HoughCircles(masked_image, cv.HOUGH_GRADIENT, dp=1.2, minDist=500, 
                              param1=50, param2=25, minRadius=10, maxRadius=15)


    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0, :]:
            if chosen is None: chosen = i
            if prevCircle is not None: 
                if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) <= dist(i[0], i[1], prevCircle[0], prevCircle[1]):
                    chosen = i


        # Draw circle around detected object
        cv.circle(img=frame, center=(200, 200), radius=190, color=(0,0,255), lineType=cv.LINE_AA, thickness=1)
        cv.circle(img=frame, center=(chosen[0], chosen[1]), radius=1, lineType=cv.LINE_AA, color=(0,0,255), thickness=2)

        # Display detected object x and y coordinate as text in image
        obejct_coordinates = f'{chosen[0].astype(int)-200}\t{chosen[1].astype(int)-200}\n'
        coordinate_display = f'({chosen[0].astype(int)-200}, {chosen[1].astype(int)-200})'
        cv.putText(img=frame, text=coordinate_display, org=(chosen[0]+20, chosen[1]), 
                   fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)

        # Output Coordinate to Serial Monitor
        return ret, np.array([chosen[0].astype(float)-200, chosen[1].astype(float)-200])
    
    cv.circle(img=frame, center=(200, 200), radius=190, color=(0,0,255), lineType=cv.LINE_AA, thickness=1)
    return ret, np.array([prevCircle[0].astype(float)-200, prevCircle[1].astype(float)-200])


def kalmanInit() -> Type[cv.KalmanFilter]:

    # Instantiate Kalman Filter Class with 
    """Keyword Arguments:
        - 4 dynamic parameters (xPos, yPos, xVel, yVel) 
        - 2 measurement parameters (xPos, yPos)
    """
    Kalman = cv.KalmanFilter(dynamParams=4, measureParams=2)

    # Transition Matrix (A) - State Transition Matrix (position & velocity)
    Kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                        [0, 1, 0, 1],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]], dtype=np.float32)
    
    # Measuremtn Matrix (C) - Measurement Matrix (position)
    Kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                         [0, 1, 0, 0]], dtype=np.float32)
    
    # Process noise covariance (Q) - Prediction Covariance
    prediction_confidence = 0.01
    Kalman.processNoiseCov = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]], dtype=np.float32) * prediction_confidence
    
    # Measurement noise covariance (R) - Measurement Covariance
    measuremnt_confidence = 10
    Kalman.measurementNoiseCov = np.array([[1, 0],
                                           [0, 1]], dtype=np.float32) * measuremnt_confidence
    
    # Error covariance matrix (P) - Posterior Covariance
    Kalman.errorCovPost = np.eye(4, dtype=np.float32)

    # Initial state (x, y, dx, dy)
    Kalman.statePost = np.array([0, 0, 0, 0], dtype=np.float32)

    return Kalman