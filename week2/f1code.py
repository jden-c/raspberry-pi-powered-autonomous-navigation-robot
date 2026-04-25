import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# --- 1. TUNING ---
BASE_SPEED = 100
TURN_SPEED = 80
U_TURN_SPEED = 50



Kp = 2.3     # Proportional — how hard to steer toward line
Kd = 6.0 	# Derivative  — dampens overcorrection, smooths frame-drop spikes

THRESHOLD_VAL = 60

SHARP_TURN_THRESHOLD = 80
SHARP_TURN_CONFIRM = 3
U_TURN_SENSITIVITY = 1700

# Max delta allowed between frames — filters frame-drop cx jumps
MAX_VALID_DELTA = 60   # pixels per frame — raise if real corners get ignored

# ROI boundaries
ROI_TOP    = 120
ROI_BOTTOM = 230
ROI_LEFT   = 40
ROI_RIGHT  = 280
ROI_CX     = (ROI_RIGHT - ROI_LEFT) // 2  # 120

# --- 2. MOTOR SETUP ---
ENA, IN1, IN2 = 25, 24, 23
IN3, IN4, ENB = 27, 17, 22

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([ENA, IN1, IN2, IN3, IN4, ENB], GPIO.OUT)

pwm_left  = GPIO.PWM(ENA, 1000)
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(0); pwm_right.start(0)

def set_motor(l_speed, r_speed):
    l_speed = max(-100, min(100, l_speed))
    r_speed = max(-100, min(100, r_speed))
    GPIO.output(IN1, GPIO.HIGH if l_speed >= 0 else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW  if l_speed >= 0 else GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH if r_speed >= 0 else GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW  if r_speed >= 0 else GPIO.HIGH)
    pwm_left.ChangeDutyCycle(abs(l_speed))
    pwm_right.ChangeDutyCycle(abs(r_speed))

def stop():
    pwm_left.ChangeDutyCycle(0); pwm_right.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

def get_roi(camera):
    img = camera.capture_array()
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    roi = gray[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT]
    _, thresh = cv2.threshold(roi, THRESHOLD_VAL, 255, cv2.THRESH_BINARY_INV)
    return thresh

def perform_turn(direction, camera):
    print(f"--- SHARP {direction.upper()} TURN ---")
    stop()
    time.sleep(0.2)

    set_motor(TURN_SPEED, -TURN_SPEED) if direction == "left" else set_motor(-TURN_SPEED, TURN_SPEED)

    timeout = time.time() + 2.0

    while time.time() < timeout:
        thresh = get_roi(camera)
        if np.sum(thresh[:, ROI_CX-10:ROI_CX+10] == 255) < 50:
            break

    while time.time() < timeout:
        thresh = get_roi(camera)
        if np.sum(thresh[:, ROI_CX-10:ROI_CX+10] == 255) > 150:
            print("Line Acquired!")
            break

    stop()
    time.sleep(0.3)

def perform_u_turn(camera):
    print("--- U-TURN ---")
    stop()
    time.sleep(0.2)
    set_motor(U_TURN_SPEED, -U_TURN_SPEED)

    timeout = time.time() + 4.0

    while time.time() < timeout:
        thresh = get_roi(camera)
        if np.sum(thresh == 255) < 300:
            break

    while time.time() < timeout:
        thresh = get_roi(camera)
        w = thresh.shape[1]
        if np.sum(thresh[:, w//4:3*w//4] == 255) > 300:
            print("U-Turn Complete!")
            break

    stop()
    time.sleep(0.3)

# --- 3. CAMERA SETUP ---
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (320, 240), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

print("--- HIGH-SPEED ARRAY ROBOT ACTIVE ---")
time.sleep(2)

prev_cx = ROI_CX
prev_error = 0          # For Kd calculation
sharp_turn_count = 0
sharp_turn_dir = None

fps_counter = 0
fps_timer = time.time()

SHOW_DISPLAY = True

try:
    while True:
        image = picam2.capture_array()

        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        roi_gray = gray[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT]
        _, thresh = cv2.threshold(roi_gray, THRESHOLD_VAL, 255, cv2.THRESH_BINARY_INV)

        roi_colour = image[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT].copy()

        roi_w = thresh.shape[1]
        left_count  = np.sum(thresh[:, 0:roi_w//6]            == 255)
        right_count = np.sum(thresh[:, roi_w - roi_w//6:]     == 255)

        # Priority 1: U-Turn
        if left_count > U_TURN_SENSITIVITY and right_count > U_TURN_SENSITIVITY:
            perform_u_turn(picam2)
            prev_cx = ROI_CX
            prev_error = 0
            sharp_turn_count = 0

        else:
            # --- THE NEW COLUMN SUM MATH ---
            # Normalize array so white pixels = 1, black = 0
            normalized_thresh = thresh / 255.0 
            
            # Sum columns vertically to get a 1D array of pixel densities
            column_sums = np.sum(normalized_thresh, axis=0)
            total_weight = np.sum(column_sums)

            # If we actually see the line
            if total_weight > 0:
                # Calculate Center of Mass (cx) using weighted average
                indices = np.arange(roi_w)
                cx = int(np.sum(indices * column_sums) / total_weight)
                delta = cx - prev_cx

                # Frame-drop filter
                if abs(delta) > MAX_VALID_DELTA:
                    print(f"[FRAME DROP FILTERED] delta={delta}")
                    cx = prev_cx  
                    delta = 0

                # Priority 2: Sharp corner
                if abs(delta) > SHARP_TURN_THRESHOLD:
                    direction = "left" if delta < 0 else "right"
                    if direction == sharp_turn_dir:
                        sharp_turn_count += 1
                    else:
                        sharp_turn_dir = direction
                        sharp_turn_count = 1

                    if sharp_turn_count >= SHARP_TURN_CONFIRM:
                        sharp_turn_count = 0
                        sharp_turn_dir = None
                        prev_error = 0
                        perform_turn(direction, picam2)
                        prev_cx = ROI_CX
                else:
                    # Priority 3: PD control
                    sharp_turn_count = 0
                    error = cx - ROI_CX
                    d_error = error - prev_error   

                    correction = (Kp * error) + (Kd * d_error)
                    set_motor(BASE_SPEED - correction, BASE_SPEED + correction)

                    prev_cx = cx
                    prev_error = error

                    if SHOW_DISPLAY:
                        cy = roi_colour.shape[0] // 2
                        cv2.circle(roi_colour, (cx, cy), 8, (0, 255, 0), -1)
                        cv2.line(roi_colour, (ROI_CX, 0), (ROI_CX, roi_colour.shape[0]), (0, 0, 255), 1)
            else:
                # Line lost — coast, don't stop
                prev_error = 0  

        # FPS
        fps_counter += 1
        if time.time() - fps_timer >= 2.0:
            print(f"FPS: {fps_counter / 2:.1f}")
            fps_counter = 0
            fps_timer = time.time()

        if SHOW_DISPLAY:
            cv2.imshow("Robot Eyes", cv2.cvtColor(roi_colour, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    pass
finally:
    stop()
    picam2.stop()
    if SHOW_DISPLAY:
        cv2.destroyAllWindows()
    GPIO.cleanup()
    print("Done.")