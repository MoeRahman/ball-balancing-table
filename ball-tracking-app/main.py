

import ball_tracking as ball
import cv2 as cv
import inverse_kinematics as ik
import math
import numpy as np
import serial
from serial.tools.list_ports import comports
import time


def main() -> None:

    videoCapture = cv.VideoCapture(1)

    arduinoSerial = serial.Serial(port='COM3', baudrate=9600, timeout=0.1)
    if(arduinoSerial.is_open): arduinoSerial.close()
    arduinoSerial.open()

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

            origin = [0, 0]
            roll_err = origin[0] - ballCoordinate[1]
            pitch_err = origin[1] - ballCoordinate[0]
            height = 40

            Kp = 0.075

            roll_out = Kp*roll_err
            pitch_out = Kp*roll_err

            servo_angles = ik.calculate_joint_angles(roll=roll_out, pitch=pitch_out, height=height)
            servoAngles = np.around(servo_angles, decimals=2)

            servoAngles = [90.0 if math.isnan(val) else val for val in servoAngles]
                
            # Serial Communication
            serial_message = ",".join([str(float(val)) for val in servoAngles])
            print(serial_message)
            arduinoSerial.write(serial_message.encode('utf-8'))
            arduinoSerial.write(b'\n')

            time.sleep(0.01)

            cv.circle(img=frame, center=(ballCoordinate[0]+200, ballCoordinate[1]+200), radius=2, color=(255,0,255), lineType=cv.LINE_AA, thickness=1)
            cv.putText(img=frame, text=coordinate_display, org=(ballCoordinate[0]+220, ballCoordinate[1]+200), 
                       fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)

        # # Outline of the platform and center frame
        cv.circle(img=frame, center=(200, 200), radius=190, color=(0,0,255), lineType=cv.LINE_AA, thickness=1)
        cv.circle(img=frame, center=(200, 200), radius=1, color=(0,0,255), lineType=cv.LINE_AA, thickness=1)

        # # DISPLAY IMAGE
        cv.imshow('video-raw', frame)

        # Quit program
        if cv.waitKey(1) & 0xFF == ord('q'):
            arduinoSerial.close()
            break

    

    videoCapture.release()
    cv.destroyAllWindows()

    return None

main()