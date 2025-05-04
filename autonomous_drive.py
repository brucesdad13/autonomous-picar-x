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

# Define HSV range for blue painter's tape
# Note: the values are in HSV color space (values may
# need to be adjusted based on lighting conditions or
# add an LED light source to the camera)
LOWER_BLUE = np.array([100, 150, 50])
UPPER_BLUE = np.array([140, 255, 255])
SERVO_MAX = 30  # Max servo angle for camera pan/tilt
        
# Utility: grab a frame and convert to proper BGR for OpenCV
def grab_bgr_frame():
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
    roi = mask[int(height * 0.75):, int(width * 0.25):int(width * 0.75)]

    # Shift contour coordinates to match full frame
    offset_x = int(width * 0.25)
    offset_y = int(height * 0.75)

    contours, _ = cv.findContours(roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None

    largest = max(contours, key=cv.contourArea)
    largest[:, 0, 0] += offset_x
    largest[:, 0, 1] += offset_y

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

#picam2.preview_configuration.main.size = (640, 480)
#picam2.preview_configuration.main.format = "BGR888"
#picam2.configure("preview")

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
lost_timer = time.time()
hopelessly_lost = False
try:
    while not hopelessly_lost:
        frame = grab_bgr_frame()  # Grab a frame from the camera
        largest, cX, cY = detect_road_contour(frame)  # Detect road contour

        if largest is not None:
            # Reset lost timer
            lost_timer = time.time()

            delta_x = cX - (frame.shape[1] // 2)
            delta_y = frame.shape[0] - cY

            angle_rad = np.arctan2(delta_x, delta_y)
            steering_angle = np.clip(np.degrees(angle_rad), -SERVO_MAX, SERVO_MAX)
            px.set_dir_servo_angle(steering_angle)
            px.forward(1)
        else:
            px.stop()
            search_angles = [-SERVO_MAX, 0, SERVO_MAX]
            found_road = False
            
            for angle in search_angles:
                px.set_cam_pan_angle(angle)
                time.sleep(0.5)
                frame = grab_bgr_frame()
                largest, cX, cY = detect_road_contour(frame)
                if largest is not None:
                    px.set_cam_pan_angle(0)  # Center camera
                    found_road = True
                    px.set_dir_servo_angle(angle)
                    px.forward(1)
                    break
            
            if found_road:
                lost_timer = time.time()
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
    music.sound_play_threading('./sounds/car-double-horn.wav') # TODO:FIXME: hardcoded path
