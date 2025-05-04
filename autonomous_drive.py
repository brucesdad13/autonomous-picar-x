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
import subprocess
from picamera2 import Picamera2
from picarx import Picarx
from robot_hat import TTS, Music
from robot_hat.utils import reset_mcu
import sys

# =============================================================================
# Initialization
# =============================================================================
reset_mcu()
px = Picarx()

# Play startup engine sound
music = Music()
music.music_set_volume(100)
music.sound_play_threading('./sounds/car-double-horn.wav') # TODO:FIXME: hardcoded path

# Setup camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "BGR888"
picam2.configure("preview")
picam2.start()
time.sleep(1)  # Let camera warm up

tts = TTS()
lost_timer = time.time()

# =============================================================================
# Main Loop
# =============================================================================
try:
    while True:
        frame = picam2.capture_array()

        # Convert to HSV and threshold for blue painter's tape
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv.inRange(hsv, lower_blue, upper_blue)

        # Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            # Reset lost timer
            lost_timer = time.time()

            # Find and draw largest contour
            largest = max(contours, key=cv.contourArea)
            M = cv.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                center_offset = cx - 320  # Frame center = 640 / 2

                # Basic steering control based on center offset
                angle = -center_offset * 0.1  # Tune coefficient
                angle = np.clip(angle, -30, 30)
                px.set_dir_servo_angle(angle)
                px.forward(1)  # Minimum safe speed

        else:
            px.stop()
            if time.time() - lost_timer > 10:
                tts.say("I'm lost. Please place me back on the road.")
                lost_timer = time.time()

        time.sleep(4)

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    px.stop()
    picam2.stop()
    subprocess.run(["aplay", "/home/charlies/picar-x/sounds/car-double-horn.wav"])
