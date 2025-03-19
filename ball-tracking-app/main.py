
import math
import cv2 as cv
import numpy as np
import serial
import ball_tracking as ball
import inverse_kinematics as ik



def main() -> None:

    videoCapture = cv.VideoCapture(0)

    arduinoSerial = serial.Serial(port='COM3', baudrate=115200, timeout=0.1)
    if(arduinoSerial.is_open): arduinoSerial.close()
    arduinoSerial.open()

    height = 50 
    roll_err = 0
    pitch_err = 0

    Kp_roll  = 0.001
    Kp_pitch = 0.001
    Kd_roll  = 0.1
    Kd_pitch = 0.1

    servo_offsets = [2, 0, -2]

    filtered_coordinates = ball.kalmanInit()
    prevPoint = np.array([0,0], dtype=np.float32)

    origin = [0, 0]
    velocity = [0, 0]


    while True:


        ret, ballCoordinate = ball.track(videoCapture)
        ret, frame = videoCapture.read()

        frame = frame[540-500:540+500, 960-500:960+500]
        frame = cv.flip(src=frame, flipCode=1)

        coordinate_measurement = np.array([ballCoordinate[0], ballCoordinate[1]], dtype=np.float32)

        if((coordinate_measurement == np.array([-500, -500])).any() == True):
            coordinate_measurement = prevPoint
        
        prevPoint = coordinate_measurement

        filtered_coordinates.predict()
        filtered_coordinates.correct(measurement=coordinate_measurement)
        predicted_position = filtered_coordinates.statePost

        if cv.waitKey(1) & 0xFF == ord('u'):
            # Update Kp Gain
            Kp_roll = float(input("Kp:\t"))
            Kp_pitch = float(input("Kp:\t"))

            # Update Kd Gain
            Kd_roll = float(input("Kd:\t"))
            Kd_pitch = float(input("Kd:\t"))
            continue

        if(len(ballCoordinate) == 0):
            pass
        else:

            obejct_coordinates = f'{ballCoordinate[0]}, {ballCoordinate[1]}\n' 
            coordinate_display = f'{ballCoordinate[0]}, {ballCoordinate[1]}'

            roll_err  = predicted_position[1] - origin[1]
            pitch_err = predicted_position[0] - origin[0]

            x_vel_error = predicted_position[3] - velocity[1]
            y_vel_error = predicted_position[2] - velocity[0]

            # Position Error Deadband
            if(roll_err < 15 and roll_err > -15): roll_err = 0
            if(pitch_err < 15 and pitch_err > -15): pitch_err = 0

            # Velocity Error Deadband
            if(x_vel_error < 2 and x_vel_error  > -2): x_vel_error = 0
            if(y_vel_error < 2 and y_vel_error  > -2): y_vel_error = 0
            
            roll_out  = Kp_roll*roll_err   + Kd_roll*x_vel_error 
            pitch_out = Kp_pitch*pitch_err + Kd_pitch*y_vel_error

            # Roll and Pitch Output Clamp
            clamp_val = 8
            if(roll_out > clamp_val): roll_out = clamp_val
            if(pitch_out > clamp_val): pitch_out = clamp_val

            clamp_val = -8
            if(roll_out < clamp_val): roll_out = clamp_val
            if(pitch_out < clamp_val): pitch_out = clamp_val

            #print("roll:\t", roll_out, "pitch:\t", pitch_out)
            
            servo_angles = ik.calculate_joint_angles(roll=roll_out, pitch=pitch_out, height=height)
            servoAngles = np.around(servo_angles, decimals=2) + servo_offsets
            print(servoAngles)
            
            servoAngles = [90.0 if math.isnan(val) else val for val in servoAngles]
                
            # Serial Communication
            serial_message = ",".join([str(float(val)) for val in servoAngles])
            #print(serial_message)
            arduinoSerial.write(serial_message.encode('utf-8'))
            arduinoSerial.write(b'\n')

            coordinate_display = f'{coordinate_measurement[0]}, {coordinate_measurement[1]}'
            cv.circle(img=frame, center=(coordinate_measurement[0].astype(int) + 500, coordinate_measurement[1].astype(int) + 500), radius=2, color=(255,0,255), lineType=cv.LINE_AA, thickness=1)
            cv.putText(img=frame, text=coordinate_display, org=(coordinate_measurement[0].astype(int)+540, coordinate_measurement[1].astype(int)+500), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)
            cv.circle(img=frame, center=(500, 500), radius=475, color=(0,255,0), lineType=cv.LINE_AA, thickness=1)
        
        coordinate_display = f'{predicted_position[0].astype(int)}, {predicted_position[1].astype(int)}'
        cv.circle(img=frame, center=(predicted_position[0].astype(int)+500, predicted_position[1].astype(int)+500), radius=2, color=(255,0,0), lineType=cv.LINE_AA, thickness=1)
        cv.putText(img=frame, text=coordinate_display, org=(predicted_position[0].astype(int)+540, predicted_position[1].astype(int)+540), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)
        cv.circle(img=frame, center=(500, 500), radius=40, color=(0,255,250), lineType=cv.LINE_AA, thickness=1)
        # Reference lines to align the platform
        for angle in range(30,275,120):
            rx = int(475*np.cos(np.deg2rad(angle)))
            ry = int(475*np.sin(np.deg2rad(angle)))
            cv.line(img=frame, pt1=(500,500), pt2=(500-rx,500-ry), color=(0,0,255), thickness=1)
        # DISPLAY IMAGE
        cv.imshow('video-raw', frame)

        # Quit program
        if cv.waitKey(1) & 0xFF == ord('q'):
            arduinoSerial.close()
            break

    

    videoCapture.release()
    cv.destroyAllWindows()

    return None

main()