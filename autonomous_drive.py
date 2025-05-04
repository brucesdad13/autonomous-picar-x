#!/usr/bin/env python3
# autonomous_drive.py

# =============================================================================
# PiCar-X Autonomous Drive Script
# - Uses PiCamera2 and OpenCV to follow blue painter's tape "road"
# - Plays startup sound
# - Moves slowly forward while adjusting direction based on contour detection
# - Falls back to "lost" mode with TTS prompt if no road is found
# =============================================================================

import cv2 as cv
import numpy as np
import time
#import subprocess
from picamera2 import Picamera2
from picarx import Picarx
from robot_hat import TTS, Music
from robot_hat.utils import reset_mcu
#import sys

# PID Tuning Parameters
Kp = 0.6  # Proportional gain
Ki = 0.05  # Integral gain
Kd = 0.3  # Derivative gain

# Define HSV range for blue painter's tape
# Note: the values are in HSV color space (values may
# need to be adjusted based on lighting conditions or
# add an LED light source to the camera)
LOWER_BLUE = np.array([90, 150, 50])
UPPER_BLUE = np.array([180, 255, 255])
SERVO_MAX = 30  # Max servo angle for camera pan/tilt
CONFIDENCE_THRESHOLD = 2  # Confidence threshold for contour detection
ANGLE_SMOOTHING_THRESHOLD = 8  # Angle smoothing threshold
MIN_CONTOUR_AREA = 1000  # Minimum contour area to be considered valid
        
# Utility: grab a frame and convert to proper BGR for OpenCV
def grab_bgr_frame():
    picam2.set_controls({"AfMode": 2})  # Trigger autofocus once
    rgb = picam2.capture_array()              # RGB
    bgr = cv.cvtColor(rgb, cv.COLOR_RGB2BGR)  # → BGR
    return bgr

def detect_road_contour(frame):
    """Given a BGR frame, isolate the road and return the largest contour (if any) and its center."""
    frame_small = cv.resize(frame, (640, 480))  # Consistent size

    # Convert from BGR to HSV
    hsv = cv.cvtColor(frame_small, cv.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors and create a bitmask
    # Note: the mask is a binary image where the blue pixels are white (255)
    # and the rest are black (0)
    mask = cv.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    # Define region of interest (ROI) as bottom quarter, middle half
    height, width = mask.shape
    roi = mask[int(height * 0.7):, :]

    # Shift contour coordinates to match full frame
    offset_x = 0 #int(width * 0.25)
    offset_y = int(height * 0.7)

    contours, _ = cv.findContours(roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None

    largest = max(contours, key=cv.contourArea)
    largest[:, 0, 0] += offset_x
    largest[:, 0, 1] += offset_y

    if cv.contourArea(largest) < MIN_CONTOUR_AREA:
        return None, None, None

    M = cv.moments(largest)
    if M["m00"] == 0:
        return None, None, None

    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return largest, cX, cY

# =============================================================================
# Initialization
# =============================================================================
reset_mcu()
px = Picarx()

# Play startup engine sound
music = Music()
music.music_set_volume(100)
music.sound_play_threading('./sounds/car-start-engine.wav') # TODO:FIXME: hardcoded path
time.sleep(1)  # Allow time for sound to play

# Setup camera
picam2 = Picamera2()
preview_config = picam2.create_still_configuration(main={"size": (3280, 2464)}) # TODO: adjust resolution for performance?
picam2.configure(preview_config)
picam2.configure(preview_config)

# Start the camera and warm-up (e.g. automatic white balance, exposure, etc.)
# The NULL preview is in fact started automatically whenever the camera
# system is started (picam2.start()) if no preview configuration is provided.
picam2.start() # Start the camera
time.sleep(2) # Allow time for the camera to adjust

# Initialize text-to-speech
tts = TTS()

# =============================================================================
# Main Loop
# =============================================================================

previous_error = 0
integral = 0
prev_angle = 0

hopelessly_lost = False
lost_timer = time.time()

try:
    while not hopelessly_lost:
        # grab and preprocess frame
        frame = cv.resize(grab_bgr_frame(), (640, 480))
        largest, cX, cY = detect_road_contour(frame)
        print(f"cX: {cX}, cY: {cY}")

        if largest is not None:
            # reset lost timer
            lost_timer = time.time()

            # compute offsets
            delta_x = cX - (640 // 2)
            delta_y = max(80, 480 - cY)

            # remember which way we should turn if ambiguous
            if abs(delta_x) > direction_threshold:
                last_direction = 1 if delta_x > 0 else -1

            # PID controller
            error = delta_x
            integral += error
            derivative = error - previous_error
            raw_angle = Kp * error + Ki * integral + Kd * derivative
            # round to nearest 2° step (i.e. multiples of 2)
            steering_angle = round(raw_angle / 2) * 2
            steering_angle = np.clip(steering_angle, -SERVO_MAX, SERVO_MAX)
            previous_error = error

            # steer & advance
            px.set_dir_servo_angle(steering_angle)
            print(f"delta_x: {delta_x}, delta_y: {delta_y}, PID output: {raw_angle:.2f}")
            print(f"Steering angle: {steering_angle:.2f}° (last_direction={last_direction})")
            px.forward(0.1)
            time.sleep(0.1)
            px.stop()

        else:
            # no contour found → search by panning & tilting
            px.stop()
            cam_pan_angles  = [-SERVO_MAX, SERVO_MAX]
            cam_tilt_angles = [-SERVO_MAX, 0]
            found_road = False

            for tilt_angle in range(cam_tilt_angles[0], cam_tilt_angles[1]+1, 10):
                px.set_cam_tilt_angle(tilt_angle)
                for pan_angle in range(cam_pan_angles[0], cam_pan_angles[1]+1, 10):
                    px.set_cam_pan_angle(pan_angle)
                    time.sleep(0.5)
                    f, x, y = detect_road_contour(grab_bgr_frame())
                    if f is not None:
                        # reset PID integrator to avoid windup
                        integral = 0
                        previous_error = 0
                        steering_angle = pan_angle
                        found_road = True
                        break
                if found_road:
                    break

            if found_road:
                lost_timer = time.time()
                px.set_cam_tilt_angle(0)
                px.set_cam_pan_angle(0)
                px.set_dir_servo_angle(steering_angle)
                print(f"Found road → steering_angle={steering_angle:.2f}°")
                px.forward(0.1)
            else:
                # give up after 10 seconds
                if time.time() - lost_timer > 10:
                    music.sound_play_threading('./sounds/car-double-horn.wav')
                    tts.say("I am hopelessly lost. Please place me back on the road.")
                    hopelessly_lost = True

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Shutting down…")

finally:
    px.stop()
    picam2.stop()

"""
try:
    Kp = 0.6  # Proportional gain
    Ki = 0.05  # Integral gain
    Kd = 0.3  # Derivative gain
    previous_error = 0
    integral = 0
    prev_angle = 0
    confidence = 0

    while not hopelessly_lost:
        frame = cv.resize(grab_bgr_frame(), (640,480))  # Grab a frame from the camera
        largest, cX, cY = detect_road_contour(frame)  # Detect road contour
        print(f"cX: {cX}, cY: {cY}")

        if largest is not None:
            # Reset lost timer
            lost_timer = time.time()

            delta_x = cX - (640 // 2)
            error = delta_x
            integral += error
            derivative = error - previous_error
            delta_y = max(80, 480 - cY)

            

            steering_angle_rad = np.arctan2(delta_x 0.1, delta_y)  # Calculate steering angle in radians
            steering_angle_deg = np.degrees(steering_angle_rad)
            smoothed_angle = round(steering_angle_deg / 2) * 2  # Round to nearest 5 degrees

            if abs(smoothed_angle - prev_angle) > ANGLE_SMOOTHING_THRESHOLD:
                confidence -= 1
            else:
                confidence += 1

            confidence = np.clip(confidence, 0, 5)

            if confidence < CONFIDENCE_THRESHOLD:
                steering_angle = np.clip(smoothed_angle, -SERVO_MAX, SERVO_MAX)
                prev_angle = steering_angle
                px.set_dir_servo_angle(steering_angle)
                print(f"delta_x: {delta_x}, delta_y: {delta_y}, angle in radians: {steering_angle_rad:.2f}, angle in degrees: {steering_angle_deg:.2f}")
                print(f"Steering angle: {steering_angle:.2f} degrees")
                px.forward(1)
                time.sleep(0.1)  # Tune this (0.1–0.25)
                px.stop()
            else:
                cam_pan_angles = [-SERVO_MAX, SERVO_MAX]
                cam_tilt_angles = [-SERVO_MAX, 0] # the road is not above the car ;)
                # Search for road by panning camera
                found_road = False
                
                for tilt_angle in range(cam_tilt_angles[0], cam_tilt_angles[1], 10):
                    px.set_cam_tilt_angle(tilt_angle) # look up/down
                    for steering_angle in range(cam_pan_angles[0], cam_pan_angles[1], 10):
                        px.set_cam_pan_angle(steering_angle)
                        time.sleep(0.5)
                        frame = grab_bgr_frame()
                        largest, cX, cY = detect_road_contour(frame)
                        if largest is not None:
                            found_road = True
                            break
                    if found_road:
                        break
                
                if found_road:
                    lost_timer = time.time()
                    px.set_cam_tilt_angle(0) # Center camera
                    px.set_cam_pan_angle(0)  # Center camera
                    px.set_dir_servo_angle(steering_angle)
                    print(f"Steering angle: {steering_angle:.2f}°")
                    px.forward(0.1)
                else:
                    if time.time() - lost_timer > 10:
                        music.sound_play_threading('./sounds/car-double-horn.wav')
                        tts.say("I am hopelessly lost. Please place me back on the road.")
                        hopelessly_lost = True
    
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    px.stop()
    picam2.stop()
    #music.sound_play_threading('./sounds/car-double-horn.wav') # TODO:FIXME: hardcoded path
"""