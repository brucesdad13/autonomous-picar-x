#!/usr/bin/env python3
# autonomous_drive.py

import cv2 as cv
import numpy as np
import time
from picamera2 import Picamera2
from picarx import Picarx
from robot_hat import TTS, Music
from robot_hat.utils import reset_mcu

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
LOWER_BLUE   = np.array([90, 50, 50])
UPPER_BLUE   = np.array([140, 255, 255])
SERVO_MAX    = 30    # servo clip in degrees
DEFAULT_TILT = -15
MIN_CONTOUR_AREA    = 1_000   # pixels
DIRECTION_THRESHOLD = 5       # px
SEARCH_STEP         = 10      # degrees

# PID gains
Kp, Ki, Kd = 0.4, 0.01, 0.05

# ──────────────────────────────────────────────────────────────────────────────
# UTILS
# ──────────────────────────────────────────────────────────────────────────────
def grab_bgr_frame():
    # trigger an autofocus, then grab & convert to BGR
    picam2.set_controls({"AfMode": 2})
    rgb = picam2.capture_array()
    return cv.cvtColor(rgb, cv.COLOR_RGB2BGR)

def detect_road_contour(frame):
    """Return (largest_contour, center_x, center_y) or (None, None, None)."""
    small = cv.resize(frame, (640, 480))
    hsv   = cv.cvtColor(small, cv.COLOR_BGR2HSV)
    mask  = cv.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    h, w = mask.shape
    roi = mask[int(h*0.7):, :]   # bottom 30%
    ox, oy = 0, int(h*0.7)

    cnts, _ = cv.findContours(roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, None

    # pick the biggest that’s big enough
    c = max(cnts, key=cv.contourArea)
    if cv.contourArea(c) < MIN_CONTOUR_AREA:
        return None, None, None

    # shift coords back to full frame
    c[:,0,0] += ox
    c[:,0,1] += oy

    M = cv.moments(c)
    if M["m00"] == 0:
        return None, None, None

    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])
    return c, cx, cy

# ──────────────────────────────────────────────────────────────────────────────
# INIT
# ──────────────────────────────────────────────────────────────────────────────
reset_mcu()
px = Picarx()
px.set_cam_tilt_angle(DEFAULT_TILT)

# startup sound
music = Music()
music.music_set_volume(30)
music.sound_play_threading('./sounds/car-start-engine.wav')
time.sleep(1)

# camera setup
picam2 = Picamera2()
cfg = picam2.create_still_configuration(main={"size": (3280, 2464)})
picam2.configure(cfg)
picam2.start()

###
picam2.set_controls({"AfMode": 2})
time.sleep(0.1)

picam2.set_controls({
    "AwbMode":          0,      # auto white balance
})
time.sleep(2)  # let the camera converge

camera_controls = {
    "AeEnable":          1,       # 1 = on
    "AeMode":            0,       # 0 = auto
    "ExposureTime":      20000,   # 20 ms
    "AnalogueGainMode":  0,       # 0 = auto gain
    "AnalogueGain":      1.0,     # no gain
    "Brightness":        0,       # 0 = no brightness
    "Contrast":          1.0,     # 0 = no contrast
    "HdrMode":           1,       # 3 = HDR mode
}

#  Read back the *actual* settled values from the metadata…
#for key in camera_controls.keys():
#    md = picam2.capture_metadata()
#    if key in md.keys():
#        print(f"{key}: {md[key]}")

#for x in picam2.capture_metadata():
#    print(x)

picam2.set_controls({
    "AwbEnable":          0,       # 1 = on
    "AwbMode":            1,       # Auto
    ##"AeEnable":           0,       # 0 = turn off auto-exposure
    "AeConstraintMode":    1,       # 0 = auto
    "AeExposureMode":       2,       # 0 = auto
    "AnalogueGainMode":   1,       # 1 = manual gain
    "AnalogueGain":       2.0,     # double the signal
    "Brightness":         0.3,       # 0 = no brightness
    #"Contrast":           1.0,       # 0 = no contrast
    "HdrMode":           3,       # 3 = HDR mode
    "NoiseReductionMode": 2,       # 0 = off
    "Saturation":         1.0,     # 0 = no saturation
    "Sharpness":          1.0,     # 0 = no sharpness
})
###

tts = TTS()

# ──────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────────────────────────────────────
# PID state
prev_err = 0
integral = 0
last_dir = 0   # +1 = right, -1 = left

lost_timer = time.time()
hopelessly_lost = False

try:
    while not hopelessly_lost:
        frame, cX, cY = grab_bgr_frame(), None, None
        cnt, cX, cY = detect_road_contour(frame)

        if cnt is not None:
            # reset lost clock
            lost_timer = time.time()

            # compute lateral error
            error   = (cX - 320) // max(80, 480 - cY)
            integral += error
            # clamp integral to avoid windup
            integral = np.clip(integral, -SERVO_MAX, SERVO_MAX)
            derivative = error - prev_err

            raw_ang = Kp*error + Ki*integral + Kd*derivative
            # deadband
            if abs(raw_ang) < DIRECTION_THRESHOLD:
                raw_ang = 0
            # keep track of sign if within small dead-zone
            if abs(error) > DIRECTION_THRESHOLD:
                last_dir = np.sign(error)

            # round & clip
            ang = round(raw_ang/2)*2
            ang = np.clip(ang, -SERVO_MAX, SERVO_MAX)
            prev_err = error

            # steer & tiny forward burst
            px.set_dir_servo_angle(int(ang))
            print(f"[DRIVE] err={error:+.0f}  → PID={raw_ang:.1f}°  → steer={ang:+.0f}°")
            px.forward(0.1)
            time.sleep(0.1)
            px.stop()

        else:
            # no contour found → sweep outwards from last known direction
            px.stop()
            found_road = False

            # Tilt scan: first look slightly down, then center
            for tilt_angle in range(-SERVO_MAX, DEFAULT_TILT, 5):  # –10° then 0°
                px.set_cam_tilt_angle(tilt_angle)

                # Build a “radial” list of pan offsets: 0, +5, –5, +10, –10, … up to SERVO_MAX
                offsets = [0]
                for d in range(5, SERVO_MAX + 1, 5):
                    offsets += [d, -d]

                # Center those offsets around last_pan (defaults to 0°)
                center = locals().get('last_pan', 0)
                for off in offsets:
                    pan = int(np.clip(center + off, -SERVO_MAX, SERVO_MAX))
                    px.set_cam_pan_angle(pan)
                    time.sleep(0.3)  # give camera exposure a moment

                    f, x, y = detect_road_contour(grab_bgr_frame())
                    if f is not None:
                        # Found the road!
                        integral = 0         # reset PID integrator
                        previous_error = 0
                        steering_angle = pan
                        found_road = True
                        last_pan = pan      # remember for next time
                        break
                if found_road:
                    break

            if found_road:
                lost_timer = time.time()
                px.set_cam_tilt_angle(0)
                px.set_cam_pan_angle(0)
                px.set_dir_servo_angle(steering_angle)
                print(f"[FOUND] pan={steering_angle:.0f}° → steering")
                px.forward(0.1)
                time.sleep(0.1)
                px.stop()
            else:
                # If still lost after 10 s, give up
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
