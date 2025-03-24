
import math
import cv2 as cv
import numpy as np
import serial
import ball_tracking as ball
import inverse_kinematics as ik



def main() -> None:

    video_capture = cv.VideoCapture(0)

    arduino_serial = serial.Serial(port='COM7', baudrate=115200, timeout=0.1)

    if arduino_serial.is_open:
        arduino_serial.close()

    arduino_serial.open()

    height = 50

    roll_err = 0
    pitch_err = 0

    cumulative_roll_err = 0
    cumulative_pitch_err = 0

    kp_roll  = 0.001
    kp_pitch = 0.001
    kd_roll  = 0.1
    kd_pitch = 0.1
    ki_roll  = 0.0001
    ki_pitch = 0.0001

    servo_offsets = [0, 2, 0]

    filtered_coordinates = ball.kalmanInit()
    prev_point = np.array([0,0], dtype=np.float32)

    position_setpoint = [0, 0]
    velocity_setpoint = [0, 0]


    while True:

        ret, ball_coordinate = ball.track(video_capture)
        ret, frame = video_capture.read()

        frame = frame[540-500:540+500, 960-500:960+500]
        frame = cv.flip(src=frame, flipCode=0)

        coordinate_measurement = np.array([ball_coordinate[0], ball_coordinate[1]], dtype=np.float32)

        if (coordinate_measurement == np.array([-500, -500])).any():
            coordinate_measurement = prev_point
        
        prev_point = coordinate_measurement

        filtered_coordinates.predict()
        filtered_coordinates.correct(measurement=coordinate_measurement)
        predicted_position = filtered_coordinates.statePost

        if cv.waitKey(1) & 0xFF == ord('u'):
            # Update Kp Gain
            kp_roll = float(input("Kp:\t"))
            kp_pitch = float(input("Kp:\t"))

            # Update Kd Gain
            kd_roll = float(input("Kd:\t"))
            kd_pitch = float(input("Kd:\t"))
            continue

        if len(ball_coordinate) == 0:
            pass
        else:

            #obejct_coordinates = f'{ball_coordinate[0]}, {ball_coordinate[1]}\n'
            coordinate_display = f'{ball_coordinate[0]}, {ball_coordinate[1]}'

            # Calculate Velocity and Position Error
            roll_err  = predicted_position[1] - position_setpoint[1]
            pitch_err = predicted_position[0] - position_setpoint[0]

            x_vel_error = predicted_position[3] - velocity_setpoint[1]
            y_vel_error = predicted_position[2] - velocity_setpoint[0]

            # Cumulative Erorr
            cumulative_roll_err  += roll_err
            cumulative_pitch_err += pitch_err

            # Position Error Deadband
            position_deadband = 20
            if(roll_err < position_deadband and roll_err > -position_deadband):
                roll_err = 0

            if(pitch_err < position_deadband and pitch_err > -position_deadband):
                pitch_err = 0

            position_outerband = 350
            if(roll_err > position_outerband or roll_err < -position_outerband):
                roll_err = 600

            if(pitch_err > position_outerband or pitch_err < -position_outerband):
                pitch_err = 600
                
            # Velocity Error Deadband
            velocity_deadband = 1
            if(x_vel_error < velocity_deadband and x_vel_error  > -velocity_deadband):
                x_vel_error = 0

            if(y_vel_error < velocity_deadband and y_vel_error  > -velocity_deadband):
                y_vel_error = 0

            # Clamp Cumulative Eror {Mitigate Integral Windup}
            integral_clamp = 50000
            if(abs(cumulative_roll_err) > integral_clamp):
                cumulative_roll_err = integral_clamp

            if(abs(cumulative_pitch_err) > integral_clamp):
                cumulative_pitch_err = integral_clamp

            # Calculate the PID Output 
            roll_out  = kp_roll*roll_err   + kd_roll*x_vel_error  + ki_roll*cumulative_roll_err
            pitch_out = kp_pitch*pitch_err + kd_pitch*y_vel_error + ki_pitch*cumulative_pitch_err 

            # Roll and Pitch Output Clamp
            clamp_val = 8
            if(abs(roll_out) > clamp_val):
                roll_out = clamp_val

            if(abs(pitch_out) > clamp_val):
                pitch_out = clamp_val

           
            servo_angles = ik.calculate_joint_angles(roll=roll_out, pitch=pitch_out, height=height)
            servo_angles = np.around(servo_angles, decimals=2) + servo_offsets

            print("Roll Output:\t", float("{:.2f}".format(roll_out)), "\tPitch Output:\t", float("{:.2f}".format(pitch_out)), 
                  "\tRoll Error:\t", float("{:.2f}".format(roll_err)), "\tPitch Output:\t", float("{:.2f}".format(pitch_err)),
                  "\tServo Angles:\t", servo_angles)
            
            # To avoid the issue with Nan values we can just round those numbers to 90 deg
            servo_angles = [90.0 if math.isnan(val) else val for val in servo_angles]
                
            # Serial Communication
            serial_message = ",".join([str(float(val)) for val in servo_angles])
            #print(serial_message)
            arduino_serial.write(serial_message.encode('utf-8'))
            arduino_serial.write(b'\n')

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
            arduino_serial.close()
            break

    

    video_capture.release()
    cv.destroyAllWindows()

    return None

main()