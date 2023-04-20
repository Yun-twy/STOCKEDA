import cv2
import time
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
import numpy as np
import threading

# Initialize PCA9685 object
pwm = PCA9685()

# Set PWM frequency and initial angles for the servos
pwm.setPWMFreq(50)
pwm.setServoPulse(1, 500) 
pwm.setRotationAngle(1, 90)
pwm.setServoPulse(0, 500) 
pwm.setRotationAngle(0, 60)

# Set camera constants
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Load the classifier
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
faces = []
# Initialize variables for servo angles and step size/delay time for servo movements
current_x_angle = 90
current_y_angle = 90
step_size = 1
delay_time = 0.1
    # Initialize camera
# cap = cv2.VideoCapture(0)
# cap.set(3, 640)
# cap.set(4, 480)

def gstreamer_pipeline(
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=True"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def face_detect():
    global faces
    window_title = "Face Detect"
    face_cascade = cv2.CascadeClassifier(
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
    )
    eye_cascade = cv2.CascadeClassifier(
        "/usr/share/opencv4/haarcascades/haarcascade_eye.xml"
    )
    video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret, frame = video_capture.read()
                frame = cv2.flip(frame,1)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, 1.3, 5)

                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    #roi_gray = gray[y : y + h, x : x + w]
                    #roi_color = frame[y : y + h, x : x + w]
                    # eyes = eye_cascade.detectMultiScale(roi_gray)
                    # for (ex, ey, ew, eh) in eyes:
                    #     cv2.rectangle(
                    #         roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2
                    # )
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

# Define the servo control function
def control_servos():
    global current_x_angle, current_y_angle, faces

    while True:
        if len(faces) > 0:
            x, y, w, h = faces[0]
            center_x = x + w//2
            center_y = y + h//2

            # Check if face is in the middle of the frame
            if abs(center_x - CAMERA_WIDTH//2) > CAMERA_WIDTH//10 or abs(center_y - CAMERA_HEIGHT//10) > CAMERA_HEIGHT//15:
				# Interpolate the angles for the servos
                x_angle = np.interp(center_x, [0, CAMERA_WIDTH], [50, 130])
                y_angle = np.interp(center_y, [0, CAMERA_HEIGHT], [70, 120])

                # Move the servos to the target angles
                for i, j in zip(range(abs(int((x_angle - current_x_angle)/step_size))), range(abs(int((y_angle - current_y_angle)/step_size)))):
                    if x_angle > current_x_angle:
                        current_x_angle += step_size
                    else:
                        current_x_angle -= step_size
                        
                    if y_angle > current_y_angle:
                        current_y_angle += step_size
                    else:
                        current_y_angle -= step_size
                    pwm.setRotationAngle(1, current_x_angle)
                    pwm.setRotationAngle(0, current_y_angle)
                    time.sleep(delay_time)
                    
if __name__ == '__main__':
    detection_thread = threading.Thread(target=face_detect)
    detection_thread.start()

    motor_thread = threading.Thread(target=control_servos)
    motor_thread.start()

    detection_thread.join()
    motor_thread.join()
