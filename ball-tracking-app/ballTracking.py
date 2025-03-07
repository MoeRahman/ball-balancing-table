

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
    prevCircle = None
    dist = lambda x1,y1,x2,y2: (x1-x2)**2+(y1-y2)**2

    ret, frame = capture.read()
    
    # Crop image to fit around platfrom plate
    frame = frame[240-200:240+200, 320-200:320+200]

    # Flip around y-axis to match platfrom coordinate frame
    frame = cv.flip(src=frame, flipCode=1)

    # Applying a Gaussian Blur to the grayscale image
    vid_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gaussian_blur = cv.GaussianBlur(vid_gray, (17, 17), 0)

    # Detect Circle using HoughCircles
    circles = cv.HoughCircles(gaussian_blur, cv.HOUGH_GRADIENT, dp=1.2, minDist=500, 
                              param1=100, param2=30, minRadius=10, maxRadius=60)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0, :]:
            if chosen is None: chosen = i
            if prevCircle is not None: 
                if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) <= dist(i[0], i[1], prevCircle[0], prevCircle[1]):
                    chosen = i

        # Draw circle around detected object
        #cv.circle(img=frame, center=(chosen[0], chosen[1]), radius=chosen[2], lineType=cv.LINE_AA, color=(0,0,255), thickness=2)
        #prevCircle=chosen

        # Display detected object x and y coordinate as text in image
        obejct_coordinates = f'{chosen[0].astype(int)-200}\t{chosen[1].astype(int)-200}\n'
        coordinate_display = f'({chosen[0].astype(int)-200}, {chosen[1].astype(int)-200})'
        cv.putText(img=frame, text=coordinate_display, org=(chosen[0]+20, chosen[1]), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)

        # Output Coordinate to Serial Monitor
        return ret, np.array([chosen[0].astype(int)-200, chosen[1].astype(int)-200])
    
    return ret, np.array([])

    # # Outline of the platform
    # cv.circle(img=frame, center=(200, 200), radius=190, color=(0,0,255), lineType=cv.LINE_AA, thickness=1)

    # # DISPLAY IMAGE
    # cv.imshow('video-raw', frame)
    # cv.imshow('processed-img', gaussian_blur)

    # return ret, 