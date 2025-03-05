

import ballTracking as ball
import cv2 as cv
import numpy as np
import inverseKinematics as ik
import serialComm as comm

def main() -> None:

    videoCapture = cv.VideoCapture(1)

    while True:
        ret, ballCoordinate = ball.track(videoCapture)
        ret, frame = videoCapture.read()
        frame = frame[240-200:240+200, 320-200:320+200]
        frame = cv.flip(src=frame, flipCode=1)

        if(len(ballCoordinate) == 0):
            pass
        else:
            obejct_coordinates = f'{ballCoordinate[0]}, {ballCoordinate[1]}\n' 
            coordinate_display = f'{ballCoordinate[0]}, {ballCoordinate[1]}'

            roll_err = 0.025*ballCoordinate[0]
            pitch_err = 0.025*ballCoordinate[1]
            height = 50

            servo_angles = ik.calculate(roll=roll_err, pitch=pitch_err, height=height)
            print(np.around(servo_angles, decimals=2))
            
            #cv.putText(img=frame, text=coordinate_display, org=(ballCoordinate[0]+220, ballCoordinate[1]+200), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)

        # # Outline of the platform
        #cv.circle(img=frame, center=(200, 200), radius=190, color=(0,0,255), lineType=cv.LINE_AA, thickness=1)

        # # DISPLAY IMAGE
        #cv.imshow('video-raw', frame)

        # Quit program
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    videoCapture.release()
    cv.destroyAllWindows()

    return None

main()