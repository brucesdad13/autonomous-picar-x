#!/usr/bin/env python3
# autonomous_drive.py
#
# MIT License
# Copyright (c) 2025 Charles K Stevenson
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# Description:
#   Continuously captures frames from the PiCamera2, detects a “road” marked
#   by blue painter’s tape (using HSV thresholding and contour extraction),
#   computes steering angles via a PID controller, and issues drive commands
#   to the PiCar-X. Includes fallback search behavior when the tape is lost
#   and audio alerts (engine start, horn, TTS).


import time
import cv2 as cv
import numpy as np
from picamera2 import Picamera2
from picarx import Picarx
from robot_hat import TTS, Music
from robot_hat.utils import reset_mcu

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────

# Tunables
Kp, Ki, Kd = 0.6, 0.05, 0.2  # PID gains
LOWER_BLUE = np.array([100, 120,  50]) # lower bound of blue in HSV
UPPER_BLUE = np.array([180, 255, 255]) # upper bound of blue in HSV
MAX_PULSE = 0.20  # maximum “straight” burst duration (s)
MIN_PULSE = 0.01  # minimum burst duration at max error
CAMERA_SETTLE_TIME = 0.01  # seconds, FIXME: impacts driving speed

# Defaults
SERVO_MAX = 30  # picar servo clip in degrees
DEFAULT_CAM_TILT = -15  # camera servo tilt angle in degrees
MIN_CONTOUR_AREA = 1_000  # pixels
DIRECTION_THRESHOLD = 5  # px
SEARCH_STEP = 10  # degrees
NATIVE_RESOLUTION = (3280, 2464)  # pixels
FRAME_RESOLUTION = (640, 480)  # pixels
FRAME_HALF = FRAME_RESOLUTION[0] // 2  # half the frame width


# ──────────────────────────────────────────────────────────────────────────────
# UTILS
# ──────────────────────────────────────────────────────────────────────────────
def grab_bgr_frame():
    """Return a BGR frame from the camera."""
    picam2.set_controls(
        {
            "AwbMode": 0,
        }
    )
    # trigger an autofocus, then grab & convert to BGR
    picam2.set_controls({"AfMode": 2})
    time.sleep(CAMERA_SETTLE_TIME)  # let the camera converge
    rgb = picam2.capture_array()
    return cv.cvtColor(rgb, cv.COLOR_RGB2BGR)


def detect_road_contour(input_frame):
    """Return (largest_contour, center_x, center_y) or (None, None, None)."""
    mask = cv.resize(input_frame, FRAME_RESOLUTION)
    mask = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
    mask = cv.inRange(mask, LOWER_BLUE, UPPER_BLUE)

    h, _ = mask.shape

    # bottom of the frame, but *stop* 10% short of the absolute bottom
    top = int(h * 0.50)
    bottom = int(h * 0.90)
    roi = mask[top:bottom, :]

    ox, oy = 0, top

    cnts, _ = cv.findContours(roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, None

    # pick the biggest that’s big enough
    c = max(cnts, key=cv.contourArea)
    if cv.contourArea(c) < MIN_CONTOUR_AREA:
        return None, None, None

    # shift coords back to full frame
    c[:, 0, 0] += ox
    c[:, 0, 1] += oy

    M = cv.moments(c)
    if not M["m00"]:
        return None, None, None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return c, cx, cy


# ──────────────────────────────────────────────────────────────────────────────
# INIT
# ──────────────────────────────────────────────────────────────────────────────
reset_mcu()  # reset the MCU to clear any previous state
px = Picarx()  # create the car object
px.set_cam_tilt_angle(DEFAULT_CAM_TILT)  # set the camera tilt angle

# Startup the sound system
music = Music()  # create the music object
music.music_set_volume(30)  # set the volume to 30%
music.sound_play_threading("./sounds/car-start-engine.wav")  # vroom!

# Camera setup
picam2 = Picamera2()  # create the camera object
cfg = picam2.create_still_configuration(main={"size": NATIVE_RESOLUTION})
picam2.configure(cfg)  # configure the camera
picam2.start()  # start the camera
picam2.set_controls(
    {
        "AfMode": 2,  # 2 = continuous
        "AwbEnable": 1,  # 1 = on
        "AwbMode": 1,  # Auto
        "AeConstraintMode": 1,  # 0 = auto
        "AeExposureMode": 2,  # 0 = auto
        "AnalogueGainMode": 0,  # 1 = manual gain
        "AnalogueGain": 2.0,  # double the signal
        "Brightness": 0.0,  # 0 = no brightness
        "Contrast": 1.0,  # 0 = no contrast
        "HdrMode": 0,  # 0 = Off, 1 = SingleExposure, 2 = MultiExposure, 3 = Night
        "NoiseReductionMode": 2,  # 0 = off
        "Saturation": 1.0,  # 0 = no saturation
        "Sharpness": 1.0,  # 0 = no sharpness
    }
)
time.sleep(CAMERA_SETTLE_TIME)  # let the camera converge

# Text-to-speech setup for announcements
tts = TTS()

# ──────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────────────────────────────────────

# PID state
prev_err = 0  # previous error
integral = 0  # integral error

# Last known direction
last_dir = 0  # +1 = right, -1 = left
last_last_dir = 0  # +1 = right, -1 = left

# Lost car state
found_road = False  # road found?
lost_timer = time.time()
hopelessly_lost = False

try:
    while not hopelessly_lost:
        frame, cX, cY = grab_bgr_frame(), None, None  # grab a frame
        cnt, cX, cY = detect_road_contour(frame)  # detect the road contour

        if cnt is not None:  # road detected
            # reset lost clock
            lost_timer = time.time()

            # PID control
            error = cX - FRAME_RESOLUTION[0] // 2  # compute lateral error
            integral += error  # accumulate error
            integral = np.clip(
                integral, -SERVO_MAX, SERVO_MAX
            )  # clamp integral to avoid windup
            derivative = error - prev_err  # compute derivative

            raw_ang = (
                Kp * error + Ki * integral + Kd * derivative
            )  # compute raw steering angle

            # deadband
            if abs(raw_ang) < DIRECTION_THRESHOLD:
                raw_ang = 0

            # keep track of sign if within small dead-zone
            if abs(error) > DIRECTION_THRESHOLD:
                last_last_dir = last_dir
                last_dir = np.sign(error)

            # round & clip
            ang = round(raw_ang / 2) * 2
            ang = np.clip(ang, -SERVO_MAX, SERVO_MAX)
            prev_err = error

            # steer & tiny forward burst
            px.set_dir_servo_angle(int(ang))
            print(
                f"[DRIVE] err={error:+.0f}  → PID={raw_ang:.1f}°  → steer={ang:+.0f}°"
            )

            # normalize error into [0…1]
            norm = min(abs(error) / FRAME_HALF, 1.0)

            # invert so norm=0→max pulse, norm=1→min pulse
            pulse = MIN_PULSE + (1.0 - norm) * (MAX_PULSE - MIN_PULSE)

            print(f"norm={norm:.2f}  pulse={pulse:.2f} s")

            # drive forward for that pulse
            px.forward(0.1)  # or whatever your base speed is
            time.sleep(pulse)
            px.stop()

        else:
            # no contour found → sweep outwards from last known direction
            px.stop()  # stop the motors
            found_road = False  # reset the found road flag

            # Tilt scan: first look slightly down, then center
            for tilt_angle in range(-SERVO_MAX, DEFAULT_CAM_TILT + 1, 15):
                px.set_cam_tilt_angle(tilt_angle)

                # Build a “radial” list of pan offsets #FIXME: wonky
                offsets = [0]
                for d in range(15, SERVO_MAX + 1, 15):
                    offsets += [d, -d]

                # Center offsets around last_pan (defaults to 0°)
                center = locals().get("last_pan", 0)
                for off in offsets:
                    pan = int(np.clip(center + off, -SERVO_MAX, SERVO_MAX))
                    px.set_cam_pan_angle(pan)
                    time.sleep(CAMERA_SETTLE_TIME)  # let the camera settle

                    f, x, y = detect_road_contour(grab_bgr_frame())  # grab a frame
                    if f is not None:
                        # Found the road!
                        integral = 0  # reset PID integrator
                        previous_error = 0  # reset previous error
                        steering_angle = pan  # steer towards the road the camera sees
                        found_road = True  # set the found road flag
                        last_pan = pan  # remember for next time
                        break  # out of the pan loop
                if found_road:
                    break  # out of the tilt loop

            if found_road:
                # Try backing up a bit to get a better view
                steer = -last_last_dir * SERVO_MAX
                px.set_dir_servo_angle(steer)
                px.backward(0.1)  # back up just a bit
                time.sleep(MAX_PULSE * 3)  # 0.6 seconds
                px.stop()  # stop the motors
                px.set_dir_servo_angle(0)  # straighten wheels
                continue  # continue to the main loop

            # still no road → try a little reverse-pulse in the opposite steer
            if last_last_dir:
                # full opposite steer
                steer = -last_last_dir * SERVO_MAX
                px.set_dir_servo_angle(steer)
                px.backward(0.1)  # back up just a bit
                time.sleep(MAX_PULSE * 3)  # 0.6 seconds
                px.stop()  # stop the motors
                px.set_dir_servo_angle(0)  # straighten wheels
                # reset your lost timer so you don’t immediately re-enter this
                lost_timer = time.time()
                continue

            # If still lost after 10 seconds, give up
            if time.time() - lost_timer > 10:
                music.sound_play_threading("./sounds/car-double-horn.wav")
                tts.say("I am hopelessly lost. Please place me back on the road.")
                hopelessly_lost = True

        time.sleep(CAMERA_SETTLE_TIME)  # FIXME: decouple from CAMERA_SETTLE_TIME

except KeyboardInterrupt:
    print("Shutting down…")

finally:
    px.stop()
    picam2.stop()
