import cv2
import numpy as np
import time
import threading
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import os

#os.environ["QT_QPA_PLATFORM"] = "xcb"



# =============================================================================
# --- 1. TUNING & CONFIGURATION ---
# =============================================================================
SHOW_DISPLAY = True
CAMERA_RES = (320, 240)

# Motor Speeds
BASE_SPEED = 40
TURN_SPEED = 80
U_TURN_SPEED = 60

# PID Control
Kp = 1.0
Kd = 2.5

# Line & Turn Detection
THRESHOLD_VAL = 80
SHARP_TURN_THRESHOLD = 60
SHARP_TURN_CONFIRM = 2
U_TURN_SENSITIVITY = 1200
MAX_VALID_DELTA = 60

# ROI Boundaries for Line Following
ROI_TOP    = 120
ROI_BOTTOM = 230
ROI_LEFT   = 40
ROI_RIGHT  = 280
ROI_CX     = (ROI_RIGHT - ROI_LEFT) // 2

# Vision System — kept strictly ABOVE the driving ROI
VISION_TOP    = 0
VISION_BOTTOM = 110

# Shape confirmation: symbol must appear this many consecutive vision frames
SHAPE_CONFIRM_FRAMES = 4
# Top-hat kernel
LINE_MAX_WIDTH = 45
kernel_tophat = np.ones((LINE_MAX_WIDTH, LINE_MAX_WIDTH), np.uint8)


# =============================================================================
# --- 3. MOTOR SETUP ---
# =============================================================================
ENA, IN1, IN2 = 25, 24, 23
IN3, IN4, ENB = 27, 17, 22

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([ENA, IN1, IN2, IN3, IN4, ENB], GPIO.OUT)

pwm_left  = GPIO.PWM(ENA, 1000)
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(0)
pwm_right.start(0)

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
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

def get_clean_roi(camera, follow_color=False):
    img  = camera.capture_array()
    roi_colour  = img[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT]
    
    if follow_color:
        hsv = cv2.cvtColor(roi_colour, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, np.array([0, 164, 100]), np.array([179, 255, 245]))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_tophat)
    else:
        gray = cv2.cvtColor(roi_colour, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, THRESHOLD_VAL, 255, cv2.THRESH_BINARY_INV)
        return cv2.morphologyEx(thresh, cv2.MORPH_TOPHAT, kernel_tophat)

def perform_turn(direction, camera, follow_color):
    print(f"--- SHARP {direction.upper()} TURN ---")
    stop()
    time.sleep(0.2)
    if direction == "left":
        set_motor(TURN_SPEED, -TURN_SPEED)
    else:
        set_motor(-TURN_SPEED, TURN_SPEED)
    timeout = time.time() + 2.0

    while time.time() < timeout:
        if np.sum(get_clean_roi(camera, follow_color)[:, ROI_CX-10:ROI_CX+10] == 255) < 50:
            break
    while time.time() < timeout:
        if np.sum(get_clean_roi(camera, follow_color)[:, ROI_CX-10:ROI_CX+10] == 255) > 150:
            print("Line acquired!")
            break
    stop()
    time.sleep(0.3)

def perform_u_turn(camera, prev_error):
    print("--- U-TURN ---")
    
    # 1. Drive forward slightly to clear the gap/junction
    set_motor(40, 40)
    time.sleep(0.6)
    
    # 2. Check memory and spin towards the last known line position
    if prev_error < 0:
#         print("Line was on the LEFT! Spinning Left.")
        set_motor(85, -85) # Your left-spin motor values
    else:
#         print("Line was on the RIGHT! Spinning Right.")
        set_motor(-85, 85) # Your right-spin motor values
        
    time.sleep(0.4) # Blind spin to get off the original line

    # 3. Setup the timeout for the safety loops
    timeout = time.time() + 4.0

    while time.time() < timeout:
        if np.sum(get_clean_roi(camera) == 255) < 300:
            break
            
    while time.time() < timeout:
        clean_thresh = get_clean_roi(camera)
        w = clean_thresh.shape[1]
        if np.sum(clean_thresh[:, w//4:3*w//4] == 255) > 300:
            print("U-Turn complete!")
            break
            
    stop()
    time.sleep(0.3)

# =============================================================================
# --- 4. VISION SYSTEM ---
# =============================================================================
def get_arrow_direction(mask, x, y, w, h):
    box = mask[y:y+h, x:x+w]
    
    # Calculate weights for all 4 halves
    top = cv2.countNonZero(box[0:h//2, :])
    bot = cv2.countNonZero(box[h//2:, :])
    lft = cv2.countNonZero(box[:, 0:w//2])
    rgt = cv2.countNonZero(box[:, w//2:])
    
    # Calculate the aspect ratio (width divided by height)
    aspect = w / float(h) if h > 0 else 1.0
    
#     # 1. Narrow on Y-axis (Short and Wide) -> Must be Left or Right
#     if aspect > 1.3:
#         
#         return "LEFT" if lft > rgt else "RIGHT"
        
    # 2. Narrow on X-axis (Tall and Thin) -> Must be Up or Down
    if aspect < 0.75:
        
        return "UP" if top > bot else "DOWN"
        
    # 3. Roughly Squarish (0.9 to 1.1) -> Fallback to the heaviest difference
    else:
        if abs(top - bot) > abs(lft - rgt):
            return "UP" if top > bot else "DOWN"
        return "LEFT" if lft > rgt else "RIGHT"


def detect_arrow(arrow_mask, annotated_image):
    """
    Detects arrows using convexity defects on thresholded contours.
    Returns direction string like 'ARROW LEFT' or None.
    """
#     _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(arrow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        if area < 800:
            continue

        x, y, w, h = cv2.boundingRect(c)
        aspect = w / float(h) if h > 0 else 0
        if not (0.5 < aspect < 1.5):
            continue
        bbox_fill = area / float(w * h) if (w * h) > 0 else 0
        if bbox_fill < 0.20:
            continue

        hull_indices = cv2.convexHull(c, returnPoints=False)
        if hull_indices is None or len(hull_indices) < 4:
            continue
        try:
            defects = cv2.convexityDefects(c, hull_indices)
        except Exception:
            continue
        if defects is None:
            continue

        min_defect_depth = 0.15 * max(w, h)
        deep_defects = []
        for i in range(defects.shape[0]):
            s, e, f, depth = defects[i, 0]
            if depth / 256.0 > min_defect_depth:
                deep_defects.append(tuple(c[f][0]))

        num_deep = len(deep_defects)
        if 1 <= num_deep <= 3:
            direction = get_arrow_direction(arrow_mask, x, y, w, h)
            label = f"ARROW {direction}"
            if SHOW_DISPLAY and annotated_image is not None:
                cv2.drawContours(annotated_image, [c], -1, (0, 255, 0), 2)
                cv2.putText(annotated_image, label, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return label

    return None


def process_vision(roi_image):

    annotated_image = roi_image.copy() if SHOW_DISPLAY else None

    hsv   = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
    gray  = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
    total_pixels = roi_image.shape[0] * roi_image.shape[1]

    # ------------------------------------------------------------------
    # 1. CAUTION — yellow card, H=20–35, bright yellow
    # ------------------------------------------------------------------
    caution_mask = cv2.inRange(hsv,np.array([8, 86, 110]), np.array([42, 255, 255]))
    caution_ratio = cv2.countNonZero(caution_mask) / total_pixels

   # print(f"Caution ratio: {caution_ratio}")
    if caution_ratio > 0.18:
#         if SHOW_DISPLAY and annotated_image is not None:
#             cv2.putText(annotated_image, "CAUTION", (5, 20),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2)
        return annotated_image, caution_mask, "CAUTION"

    # ------------------------------------------------------------------
    # 2. FINGER — purple/violet arcs, H=130–160
    # ------------------------------------------------------------------
    finger_mask = cv2.inRange(hsv, np.array([122, 52, 64]), np.array([152, 255, 255]))
    finger_ratio = cv2.countNonZero(finger_mask) / total_pixels
#     print(f"finger ratio: {finger_ratio}")

    if finger_ratio > 0.01:
#         if SHOW_DISPLAY and annotated_image is not None:
#             cv2.putText(annotated_image, "FINGER", (5, 20),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 200), 2)
        return annotated_image, finger_mask, "FINGER"

    # ------------------------------------------------------------------
    # 3. QR — purple/violet arcs, H=130–160
    # ------------------------------------------------------------------
    qr_mask = cv2.inRange(hsv, np.array([97, 52, 64]), np.array([122, 255, 255]))
    qr_ratio = cv2.countNonZero(qr_mask) / total_pixels
#     print(f"qr ratio: {qr_ratio}")
    
    if qr_ratio > 0.05:
#         if SHOW_DISPLAY and annotated_image is not None:
#             cv2.putText(annotated_image, "FINGER", (5, 20),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 200), 2)
        return annotated_image, qr_mask, "QR"


    # ------------------------------------------------------------------
    # 4. GREENHAND — large dark-green filled rectangle
    # ------------------------------------------------------------------
    greenhand_mask = cv2.inRange(hsv, np.array([52, 43, 33]), np.array([105, 255, 255]))
    greenhand_ratio = cv2.countNonZero(greenhand_mask) / total_pixels
    
#     print(f"Greenhand ratio: {greenhand_ratio}")
    if greenhand_ratio > 0.14:
        return annotated_image, greenhand_mask, "GREENHAND"
    
# ------------------------------------------------------------------
    # 5. RECYCLE ORB - The "Donut Hole" Test
    # ------------------------------------------------------------------
    recycle_mask = cv2.inRange(hsv, np.array([50, 60, 69]), np.array([85, 255, 255]))
    recycle_pixels = cv2.countNonZero(recycle_mask)
    
    
    # If we see enough green to be a sign (Arrow is 1100, Recycle is 2000, so >400 is safe)
    if recycle_pixels > 400: 
        # Find the giant box that surrounds ALL the green pixels
        points = cv2.findNonZero(recycle_mask)
        if points is not None:
            rx, ry, rw, rh = cv2.boundingRect(points)
            
            # 1. Size Check (Ignore random floor noise)
            if rw < 160 and rh < 160:
                
                # 2. THE DONUT HOLE TEST
                # Find the exact center coordinates of the sign
                cx = rx + (rw // 2)
                cy = ry + (rh // 2)
                
                # Snip a tiny 10x10 pixel square directly out of the middle
                center_slice = recycle_mask[max(0, cy-5):cy+5, max(0, cx-5):cx+5]
                center_pixels = cv2.countNonZero(center_slice)
                
                # If the middle is completely empty paper (< 10 green pixels)...
                # It is impossible to be an Arrow! It must be the hollow Recycle sign!
                if center_pixels < 10:
                    if SHOW_DISPLAY and annotated_image is not None:
                        cv2.rectangle(annotated_image, (rx, ry), (rx+rw, ry+rh), (0, 255, 255), 2)
                        cv2.putText(annotated_image, f"RECYCLE ({recycle_pixels}px)", (rx, max(ry - 10, 0)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 100), 2)
                    
                    return annotated_image, recycle_mask, "RECYCLE"

    # ------------------------------------------------------------------
    # 6. ARROW — convexity defects (1–3 deep defects)
    # ------------------------------------------------------------------
    arrow_mask = cv2.inRange(hsv, np.array([0, 65, 81]), np.array([179, 255, 255]))
#     arrow_ratio = cv2.countNonZero(arrow_mask) / total_pixels
#     print(arrow_ratio)
    arrow_result = detect_arrow(arrow_mask, annotated_image)
    if arrow_result:
        return annotated_image, arrow_mask, arrow_result

    return annotated_image, gray, "NONE"


# =============================================================================
# --- 5. THREADED VISION SYSTEM ---
# =============================================================================
_vision_lock  = threading.Lock()
# To this:
_vision_state = {
    "label":      "NONE",
    "annotated":  None,
    "mask":       None,   # <--- Add this!
    "running":    True,
}

_shape_votes = {}

def _vision_worker(camera):
    global _shape_votes
    while True:
        with _vision_lock:
            if not _vision_state["running"]:
                break

        frame = camera.capture_array()
        vision_slice = frame[VISION_TOP:VISION_BOTTOM, 0:320]
#         bgr_slice    = cv2.cvtColor(vision_slice, cv2.COLOR_RGB2BGR)

        annotated, debug_mask, raw_label = process_vision(vision_slice)

        if raw_label != "NONE":
            _shape_votes[raw_label] = _shape_votes.get(raw_label, 0) + 1
        else:
            _shape_votes.clear()

        confirmed_label = "NONE"
        for lbl, count in _shape_votes.items():
            if count >= SHAPE_CONFIRM_FRAMES:
                confirmed_label = lbl
                break

        with _vision_lock:
#             _vision_state["label"]     = confirmed_label
            _vision_state["label"]     = raw_label
            _vision_state["annotated"] = annotated
            _vision_state["mask"]      = debug_mask  # <--- Add this!

def start_vision_thread(camera):
    t = threading.Thread(target=_vision_worker, args=(camera,), daemon=True)
    t.start()
    return t


# =============================================================================
# --- 6. MAIN EXECUTION LOOP ---
# =============================================================================
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": CAMERA_RES, "format": "RGB888"},
    controls={"FrameDurationLimits": (16667, 16667)}  # 60fps
)
picam2.configure(config)
picam2.start()
time.sleep(2)

picam2.set_controls({
    "AeEnable": False,  # Turns off Auto Exposure (brightness)
    "AwbEnable": False  # Turns off Auto White Balance (color shifting)
})

print("Camera brightness and colors LOCKED!")

print("\n--- ROBOT BRAIN ACTIVE (colour-based vision) ---")

prev_cx = ROI_CX
prev_error = 0
sharp_turn_count = 0
sharp_turn_dir   = None

fps_counter = 0
fps_timer   = time.time()

last_symbol     = "NONE"
symbol_cooldown = 0
resume_time     = 0
clean_thresh    = None

recycle_count = 0

# --- SHORTCUT MEMORY VARIABLES ---
on_shortcut = False
shortcut_exit_turn = "right" # Fallback default
last_known_turn = "right"    # Tracks the most recent sharp turn
# ---------------------------------

vision_thread = start_vision_thread(picam2)

try:
    while True:
        with _vision_lock:
            status_label = _vision_state["label"]
            result_view  = _vision_state["annotated"]
            vision_mask  = _vision_state["mask"]
        if status_label != "NONE" and time.time() > symbol_cooldown:
#             if status_label != last_symbol:
            print(f">>> SYMBOL CONFIRMED: {status_label}")
            last_symbol     = status_label
            symbol_cooldown = time.time() + 2.0  # longer than any action duration

            if status_label != "RECYCLE":
                recycle_count = 0

            if status_label in ("CAUTION", "GREENHAND"):
                print("confirm greenhand")
                stop()
                time.sleep(2)
                set_motor(40, 40)
                time.sleep(0.6)
                stop()

            elif status_label == "RECYCLE":
                if recycle_count > 3:
                    print("confirm recycle")
                    recycle_count = 0
                    stop()
                    time.sleep(0.5)
                    
                    # Phase 1: blind spin get away from original line
                    set_motor(60, -60)
                    time.sleep(1.9)  # tune this just enough to clear the original line
                    
                    # Phase 2: keep spinning but now check for new line
                    timeout = time.time() + 3.0  # safety timeout so it doesn't spin forever
                    while time.time() < timeout:
                        line_view = get_clean_roi(picam2)
                        center_strip = line_view[:, ROI_CX-15:ROI_CX+15]
                        if np.sum(center_strip == 255) > 100:  # line detected in centre
                            print("New line acquired after recycle spin!")
                            break
                    
                    stop()
                    time.sleep(0.2)
                    set_motor(40, 40)
                    time.sleep(0.4)
                    stop()
                else:
                    recycle_count += 1
            elif status_label in {"ARROW RIGHT", "ARROW LEFT"}:
                if status_label == "ARROW RIGHT":
                    set_motor(40,40)
                    time.sleep(0.6)
                    set_motor(-85, 85)
                    time.sleep(0.2)
                    
                    # Keep spinning right while looking for a new line
                    timeout = time.time() + 3.0
                    while time.time() < timeout:
                        line_view = get_clean_roi(picam2)
                        center_strip = line_view[:, ROI_CX-15:ROI_CX+15]
                        if np.sum(center_strip == 255) > 100:
                          
                            print("New line acquired after RIGHT arrow spin!")
                            stop()
                            time.sleep(0.2)
                            break
                        
                        set_motor(-TURN_SPEED, TURN_SPEED)  # keep spinning right
                    
                    stop()
                    time.sleep(0.2)
                    set_motor(40, 40)
                    time.sleep(0.4)
                    stop()

                elif status_label == "ARROW LEFT":
                    set_motor(40,40)
                    time.sleep(0.6)
                    set_motor(85, -85)
                    time.sleep(0.4)
                    
                    # Keep spinning left while looking for a new line
                    timeout = time.time() + 3.0
                    while time.time() < timeout:
                        line_view = get_clean_roi(picam2)
                        center_strip = line_view[:, ROI_CX-15:ROI_CX+15]
                        if np.sum(center_strip == 255) > 100:
#                             print(f"White pixels: {white_count}")
                            print("New line acquired after LEFT arrow spin!")
                            stop()
                            time.sleep(0.5)
                            break
                        set_motor(TURN_SPEED, -TURN_SPEED)  # keep spinning left
                    
                    stop()
                    time.sleep(0.2)
                    set_motor(40, 40)
                    time.sleep(0.4)
                    stop()
                    
            else:
                print("drive straight arow up")
                set_motor(40,40)
                time.sleep(0.5)
                stop()
                time.sleep(0.2)
                    
                    
# ------------------------------------------------------------------
        # LINE FOLLOWING & MOTOR CONTROL
        # ------------------------------------------------------------------
        frame      = picam2.capture_array()
        roi_colour = frame[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT].copy()

        if time.time() < resume_time:
            # stop()
            prev_error = 0
            if SHOW_DISPLAY and result_view is not None:
                cv2.putText(result_view, "PAUSED: IDENTIFYING...", (5, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            # 1. Standard Black Line Mask
            gray_roi = cv2.cvtColor(roi_colour, cv2.COLOR_RGB2GRAY)
            _, raw_thresh = cv2.threshold(gray_roi, THRESHOLD_VAL, 255, cv2.THRESH_BINARY_INV)
            black_thresh = cv2.morphologyEx(raw_thresh, cv2.MORPH_TOPHAT, kernel_tophat)
            
            # 2. Colored Shortcut Mask (Red/Yellow)
            hsv_roi = cv2.cvtColor(roi_colour, cv2.COLOR_RGB2HSV)
            color_thresh = cv2.inRange(hsv_roi, np.array([0, 143, 148]), np.array([179, 255, 255]))
#             color_thresh = cv2.morphologyEx(color_mask_raw, cv2.MORPH_OPEN, kernel_tophat)
# 3. Count the pixels
            black_pixels = cv2.countNonZero(black_thresh)
            color_pixels = cv2.countNonZero(color_thresh)

            # --- 4. TAPE VS CARD FILTER ---
            # We must ignore red Arrow cards on the floor!
            is_real_shortcut = False
            
            if color_pixels > 300:
                color_points = cv2.findNonZero(color_thresh)
                if color_points is not None:
                    cx, cy, cw, ch = cv2.boundingRect(color_points)
                    
                    # A continuous tape line will stretch across the ROI.
                    # ROI height is 110. If the blob height is > 85, it's a vertical tape.
                    # ROI width is 240. If the blob width is > 120, it's a horizontal branching tape.
                    # Arrow cards are small squares (e.g. 60x60), so they will fail this test!
                    if ch > 85 or cw > 120:
                        is_real_shortcut = True

            # --- SHORTCUT ENTRY & EXIT LOGIC ---
            if is_real_shortcut:
                clean_thresh = color_thresh
                if not on_shortcut:
                    on_shortcut = True
                    
                    # --- NEW MEMORY LOGIC: The "Bumper" Slice ---
                    h, w = black_thresh.shape
                    
                    # We ONLY look at the bottom 40% of the image (closest to the tires)
                    bottom_start = int(h * 0.6)
                    
                    bottom_black = black_thresh[bottom_start:, :]
                    bottom_color = color_thresh[bottom_start:, :]
                    
                    # 1. Find the exact X-center of the Black Line near the bumper
                    black_cols = np.sum(bottom_black / 255.0, axis=0)
                    b_weight = np.sum(black_cols)
                    black_cx = int(np.sum(np.arange(w) * black_cols) / b_weight) if b_weight > 0 else w // 2
                    
                    # 2. Find the exact X-center of the Colored Line near the bumper
                    color_cols = np.sum(bottom_color / 255.0, axis=0)
                    c_weight = np.sum(color_cols)
                    color_cx = int(np.sum(np.arange(w) * color_cols) / c_weight) if c_weight > 0 else w // 2
                    
                    # 3. Compare the stems of the lines!
                    if color_cx > black_cx:
                        shortcut_exit_turn = "right"
                    else:
                        shortcut_exit_turn = "left"
                    
                    print(f"--- FORK DETECTED! Bumper Red CX:{color_cx} vs Black CX:{black_cx} -> Memorized: {shortcut_exit_turn.upper()} ---")
            else:
                clean_thresh = black_thresh
                
                # If we were on a shortcut, and black suddenly overtakes color...
                if on_shortcut and black_pixels > color_pixels + 300:
                    print(f"--- EXITING SHORTCUT! Forcing {shortcut_exit_turn.upper()} turn ---")
                    perform_turn(shortcut_exit_turn, picam2, follow_color=False)
                    on_shortcut = False
                    prev_cx = ROI_CX
                    prev_error = 0
                    continue # Skip the rest of this frame to start fresh
            # -----------------------------------
            roi_w = clean_thresh.shape[1]
            left_count  = np.sum(clean_thresh[:, 0:roi_w//6] == 255)
            right_count = np.sum(clean_thresh[:, roi_w - roi_w//6:] == 255)

            if left_count > U_TURN_SENSITIVITY and right_count > U_TURN_SENSITIVITY:
                perform_u_turn(picam2, prev_error)
                prev_cx = ROI_CX; prev_error = 0; sharp_turn_count = 0
            else:
                normalized_thresh = clean_thresh / 255.0
                column_sums  = np.sum(normalized_thresh, axis=0)
                total_weight = np.sum(column_sums)

                if total_weight > 0:
                    indices = np.arange(roi_w)
                    cx      = int(np.sum(indices * column_sums) / total_weight)
                    delta   = cx - prev_cx

                    if abs(delta) > MAX_VALID_DELTA:
                        cx = prev_cx
                        delta = 0

                    if abs(delta) > SHARP_TURN_THRESHOLD:
                        direction = "left" if delta < 0 else "right"
                        if direction == sharp_turn_dir:
                            sharp_turn_count += 1
                        else:
                            sharp_turn_dir   = direction
                            sharp_turn_count = 1

                        if sharp_turn_count >= SHARP_TURN_CONFIRM:
                            sharp_turn_count = 0
                            sharp_turn_dir   = None
                            prev_error       = 0
                            last_known_turn = direction
                            perform_turn(direction, picam2, follow_color=on_shortcut)
                            prev_cx = ROI_CX
                    else:
                        sharp_turn_count = 0
                        error      = cx - ROI_CX
                        correction = (Kp * error) + (Kd * (error - prev_error))
                        set_motor(BASE_SPEED - correction, BASE_SPEED + correction)
                        prev_cx    = cx
                        prev_error = error

                        if SHOW_DISPLAY:
                            cv2.circle(roi_colour, (cx, roi_colour.shape[0]//2), 8, (0, 255, 0), -1)
                            cv2.line(roi_colour, (ROI_CX, 0), (ROI_CX, roi_colour.shape[0]), (0, 0, 255), 1)
                else:
                    # ---> AUTO-RECOVERY WHEN LINE IS COMPLETELY LOST <---
#                     print("LINE LOST! Auto-recovering...")
                    if prev_error < 0:
#                         print("Line was on the LEFT! Spinning Left.")
                        set_motor(70, -70) # Spin Left to find it
                    else:
#                         print("Line was on the RIGHT! Spinning Right.")
                        set_motor(-70, 70) # Spin Right to find it

        # ------------------------------------------------------------------
        # METRICS & DISPLAY
        # ------------------------------------------------------------------
        fps_counter += 1
        if time.time() - fps_timer >= 2.0:
            print(f"FPS: {fps_counter / 2:.1f}")
            fps_counter = 0
            fps_timer   = time.time()

        if SHOW_DISPLAY:
            if clean_thresh is not None:
                cv2.imshow("1. Clean Driving Line", clean_thresh)

            if result_view is not None:
                color = (0, 255, 0) if status_label != "NONE" else (0, 0, 255)
                cv2.putText(result_view, f"STATUS: {status_label}", (5, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.imshow("2. Vision Target", result_view)
                

                    
                
            if vision_mask is not None:
                cv2.imshow("4. Color Mask Debug", vision_mask)

#             cv2.imshow("3. Robot Eyes", cv2.cvtColor(roi_colour, cv2.COLOR_RGB2BGR))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    pass

finally:
    with _vision_lock:
        _vision_state["running"] = False
    stop()
    picam2.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    print("Done.")






