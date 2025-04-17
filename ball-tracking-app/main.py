
import math
import cv2 as cv
from cv2 import VideoCapture
import numpy as np
from PyQt5 import QtWidgets
import serial
import sys
import threading
import time
from joystick import XboxController 
import ball_tracking as ball
from controls_analysis_tool import ControlsWindow
import inverse_kinematics as ik

height = 50
roll_err = 0
pitch_err = 0

x_vel_error = 0
y_vel_error = 0

alpha = 0.9
prev_x_vel_error = 0
prev_y_vel_error = 0

cumulative_roll_err = 0
cumulative_pitch_err = 0

kp_roll  = 0.02
kp_pitch = 0.02
kd_roll  = 0.1
kd_pitch = 0.1
ki_roll  = 0
ki_pitch = 0
servo_offsets = [0, 0, 0]

filtered_coordinates = ball.kalmanInit()
prev_point = np.array([0,0], dtype=np.float32)

def ball_balance(plot_window: ControlsWindow, controller: XboxController, video_capture: VideoCapture, arduino_serial: serial.Serial) -> None:

    global height, roll_err, pitch_err
    global x_vel_error, y_vel_error
    global alpha, prev_x_vel_error, prev_y_vel_error
    global cumulative_roll_err, cumulative_pitch_err
    global kp_roll, kp_pitch, kd_roll, kd_pitch, ki_roll, ki_pitch
    global servo_offsets
    global filtered_coordinates
    global prev_point
    total_time = 0

    while True:
        start_time = time.time()

        position_setpoint = [200 * controller.read()[0], 200 * controller.read()[1]]
        reset_position = controller.read()[2]

        ret, ball_coordinate = ball.track(video_capture)
        ret, frame = video_capture.read()

        frame = frame[540-500:540+500, 960-500:960+500]
        frame = cv.flip(src=frame, flipCode=0)
        
        coordinate_measurement = np.array([ball_coordinate[0], ball_coordinate[1]], dtype=np.float32)

        if(reset_position == 1):
            coordinate_measurement = np.array([0,0], dtype=np.float32)
        
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

        #obejct_coordinates = f'{ball_coordinate[0]}, {ball_coordinate[1]}\n'
        coordinate_display = f'{ball_coordinate[0]}, {ball_coordinate[1]}'

        # Calculate Velocity and Position Error
        roll_err  = predicted_position[1] - position_setpoint[1]
        pitch_err = predicted_position[0] - position_setpoint[0]

        x_vel_error = predicted_position[3]
        y_vel_error = predicted_position[2]

        # Position Error Deadband
        position_deadband = 35
        if(roll_err < position_deadband and roll_err > -position_deadband):
            roll_err = 0
            cumulative_roll_err = cumulative_roll_err

        if(pitch_err < position_deadband and pitch_err > -position_deadband):
            pitch_err = 0
            cumulative_pitch_err = cumulative_pitch_err

        # Resolving Derivative/Velocity Kick by filtering velocity signal
        filtered_x_vel_error = alpha * x_vel_error + (1 - alpha) * prev_x_vel_error 
        filtered_y_vel_error = alpha * y_vel_error + (1 - alpha) * prev_y_vel_error

        prev_x_vel_error = filtered_x_vel_error
        prev_y_vel_error = filtered_y_vel_error

        # Velocity Error Deadband
        velocity_deadband = 2
        if(filtered_x_vel_error < velocity_deadband and filtered_x_vel_error > -velocity_deadband):
            filtered_x_vel_error = 0

        if(filtered_y_vel_error < velocity_deadband and filtered_y_vel_error > -velocity_deadband):
            filtered_y_vel_error = 0

        # Velocity Error Clamp
        velocity_clamp = 60
        if(filtered_x_vel_error > velocity_clamp):
            filtered_x_vel_error = velocity_clamp

        if(filtered_x_vel_error < -1*velocity_clamp):
            filtered_x_vel_error = -1*velocity_clamp

        if(filtered_y_vel_error > velocity_clamp):
            filtered_y_vel_error = velocity_clamp

        if(filtered_y_vel_error < -1*velocity_clamp):
            filtered_y_vel_error = -1*velocity_clamp

        # Cumulative Erorr
        cumulative_roll_err  += roll_err
        cumulative_pitch_err += pitch_err

        # Clamp Cumulative Eror {Mitigate Integral Windup}
        integral_clamp = 20000
        if(abs(cumulative_roll_err) > integral_clamp):
            if(cumulative_roll_err < 0):
                cumulative_roll_err = -1*integral_clamp
            else:
                cumulative_roll_err = integral_clamp

        if(abs(cumulative_pitch_err) > integral_clamp):
            if(cumulative_pitch_err == 0):
                cumulative_pitch_err = -1*integral_clamp
            else:
                cumulative_pitch_err = integral_clamp

        # Calculate the PID Output 
        roll_out  = kp_roll*roll_err   + kd_roll *filtered_x_vel_error + ki_roll*cumulative_roll_err
        pitch_out = kp_pitch*pitch_err + kd_pitch*filtered_y_vel_error + ki_pitch*cumulative_pitch_err 

        # Roll and Pitch Output Clamp
        clamp_val = 10
        if(abs(roll_out) > clamp_val):
            roll_out = clamp_val

        if(abs(pitch_out) > clamp_val):
            pitch_out = clamp_val
        
        servo_angles = ik.calculate_joint_angles(roll=roll_out, pitch=pitch_out, height=height)
        servo_angles = np.around(servo_angles, decimals=2) + servo_offsets

        # To avoid the issue with Nan values we can just round those numbers to 90 deg
        servo_angles = [90.0 if math.isnan(val) else val for val in servo_angles]
            
        # Serial Communication
        serial_message = ",".join([str(float(val)) for val in servo_angles])
        arduino_serial.write(serial_message.encode('utf-8'))
        arduino_serial.write(b'\n')

        coordinate_display = f'{coordinate_measurement[0]}, {coordinate_measurement[1]}'

        cv.circle(img=frame, center=(int(position_setpoint[0]) + 500, int(position_setpoint[1]) + 500), radius=40, color=(0,0,255), lineType=cv.LINE_AA, thickness=2)
        cv.circle(img=frame, center=(500, 500), radius=450, color=(0,0,0), lineType=cv.LINE_AA, thickness=1)
        cv.circle(img=frame, center=(coordinate_measurement[0].astype(int) + 500, coordinate_measurement[1].astype(int) + 500), radius=2, color=(255,0,255), lineType=cv.LINE_AA, thickness=2)
        cv.putText(img=frame, text=coordinate_display, org=(coordinate_measurement[0].astype(int)+540, coordinate_measurement[1].astype(int)+500), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)

        coordinate_display = f'{predicted_position[0].astype(int)}, {predicted_position[1].astype(int)}'

        cv.circle(img=frame, center=(predicted_position[0].astype(int)+500, predicted_position[1].astype(int)+500), radius=2, color=(255,0,0), lineType=cv.LINE_AA, thickness=2)
        cv.putText(img=frame, text=coordinate_display, org=(predicted_position[0].astype(int)+540, predicted_position[1].astype(int)+540), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, color=(0, 0, 0), lineType=cv.LINE_AA, thickness=1)

        end_time = time.time()
        total_time += end_time - start_time

        # Plot data for controls analysis
        # TODO add pause button feature to view plot at critical moments
        plot_window.update_data(total_time, plot1 = [position_setpoint[0], predicted_position[0]],
                                            plot2 = [position_setpoint[1], predicted_position[1]],
                                            plot3 = [filtered_x_vel_error, filtered_y_vel_error],
                                            plot4 = [roll_out, pitch_out])

        # Reference lines to align the platform
        for angle in range(30,275,120):
            rx = int(475*np.cos(np.deg2rad(angle)))
            ry = int(475*np.sin(np.deg2rad(angle)))
            cv.line(img=frame, pt1=(500,500), pt2=(500-rx,500-ry), color=(0,0,0), thickness=1)


        frame = cv.resize(frame, (500, 500))
        # DISPLAY IMAGE
        cv.imshow('Top_Down_View', frame)

        # Quit program
        if cv.waitKey(1) & 0xFF == ord('q'):
            arduino_serial.close()
            break

    

    video_capture.release()
    cv.destroyAllWindows()

    return None


def main():

    app = QtWidgets.QApplication(sys.argv)
    plot_window = ControlsWindow()
    plot_window.show()

    controller = XboxController()

    video_capture = VideoCapture(0)

    arduino_serial = serial.Serial(port='COM13', baudrate=115200, timeout=0.1)

    if arduino_serial.is_open:
        arduino_serial.close()

    arduino_serial.open()

    # Run control loop in a thread
    control_thread = threading.Thread(target=ball_balance, args=(plot_window,controller,video_capture,arduino_serial))
    control_thread.start()

    sys.exit(app.exec_())


main()