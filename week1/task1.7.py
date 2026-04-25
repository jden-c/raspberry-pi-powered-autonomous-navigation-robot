import RPi.GPIO as GPIO
import time

# ==========================================
# --- USER TUNING SECTION (EDIT THIS) ---
# ==========================================

# 1. STRAIGHT DRIVING
LEFT_SPEED_MOVE  = 75   
RIGHT_SPEED_MOVE = 75   

# 2. SCENARIO TUNING (The "Forward" vs "Reverse" side logic)

# --- SCENARIO 1: PRECISION (-90 to 90) ---
# When turning:
# The Forward-moving wheels get speed 75.
# The Backward-moving wheels get speed 65.
PRECISION_SPEED_FWD_SIDE = 90   
PRECISION_SPEED_REV_SIDE = 90   
TIME_360_PREC            = 2.6  

# --- SCENARIO 2: FAST (-270 to -91, 91 to 270) ---
FAST_SPEED_FWD_SIDE      = 80
FAST_SPEED_REV_SIDE      = 75
TIME_360_FAST            = 2.31  

# --- SCENARIO 3: SPIN (271 to 360) ---
SPIN_SPEED_FWD_SIDE      = 85
SPIN_SPEED_REV_SIDE      = 85
TIME_360_SPIN            = 2.15 

# ==========================================

# --- PIN CONFIGURATION ---
ENA, IN1, IN2 = 25, 27, 17
IN3, IN4, ENB = 24, 23, 22

# --- SETUP ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([ENA, IN1, IN2, IN3, IN4, ENB], GPIO.OUT)

pwm_left = GPIO.PWM(ENA, 1000)
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(0)
pwm_right.start(0)

# --- MOTOR FUNCTION ---
def set_motor(left_speed, right_speed, left_dir, right_dir):
    # Left Side
    GPIO.output(IN1, GPIO.HIGH if left_dir == 1 else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW if left_dir == 1 else GPIO.HIGH)
    pwm_left.ChangeDutyCycle(left_speed)
    
    # Right Side
    GPIO.output(IN3, GPIO.HIGH if right_dir == 1 else GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW if right_dir == 1 else GPIO.HIGH)
    pwm_right.ChangeDutyCycle(right_speed)

def stop():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

def turn_precise(angle):
    mag = abs(angle)
    
    # --- 1. DETERMINE SPEEDS BASED ON SCENARIO ---
    if mag <= 90:
        # Precision Mode (Asymmetric: 75 vs 65)
        spd_fwd = PRECISION_SPEED_FWD_SIDE
        spd_rev = PRECISION_SPEED_REV_SIDE
        active_calib = TIME_360_PREC
        print(f">>> PRECISION | FwdSide:{spd_fwd} RevSide:{spd_rev}")
        
    elif mag <= 270:
        # Fast Mode (Symmetric: 80 vs 80)
        spd_fwd = FAST_SPEED_FWD_SIDE
        spd_rev = FAST_SPEED_REV_SIDE
        active_calib = TIME_360_FAST
        print(f">>> FAST | FwdSide:{spd_fwd} RevSide:{spd_rev}")
        
    else:
        # Spin Mode (Symmetric: 85 vs 85)
        spd_fwd = SPIN_SPEED_FWD_SIDE
        spd_rev = SPIN_SPEED_REV_SIDE
        active_calib = TIME_360_SPIN
        print(f">>> SPIN | FwdSide:{spd_fwd} RevSide:{spd_rev}")
    
    # --- 2. ASSIGN SPEEDS BASED ON DIRECTION ---
    if angle > 0:
        # RIGHT TURN: 
        # Left side moves FORWARD (Needs 75)
        # Right side moves BACKWARD (Needs 65)
        final_left_speed  = spd_fwd
        final_right_speed = spd_rev
        dir_l, dir_r = 1, 0
        
    else:
        # LEFT TURN:
        # Left side moves BACKWARD (Needs 65)
        # Right side moves FORWARD (Needs 75)
        final_left_speed  = spd_rev
        final_right_speed = spd_fwd
        dir_l, dir_r = 0, 1

    # --- 3. CALCULATE DURATION ---
    sec_per_deg = active_calib / 360.0
    duration = mag * sec_per_deg
    
    print(f"    Target: {angle}° | L:{final_left_speed} R:{final_right_speed} | Time:{duration:.3f}s")
    
    # --- 4. MOVE ---
    set_motor(final_left_speed, final_right_speed, dir_l, dir_r)
    time.sleep(duration)
    stop()

# --- MAIN CONTROLLER ---
print("\n=== CORRECTED LOGIC ROBOT ===")
print("When turning precise:")
print(f"Forward Wheel = {PRECISION_SPEED_FWD_SIDE}")
print(f"Reverse Wheel = {PRECISION_SPEED_REV_SIDE}")

try:
    while True:
        cmd = input("> ").lower().strip()
        
        if cmd == 'w': 
            set_motor(LEFT_SPEED_MOVE, RIGHT_SPEED_MOVE, 1, 1)
            time.sleep(10); stop()
        elif cmd == 's': 
            set_motor(LEFT_SPEED_MOVE, RIGHT_SPEED_MOVE, 0, 0)
            time.sleep(10); stop()
        elif cmd == 'a': 
            set_motor(FAST_SPEED_REV_SIDE, FAST_SPEED_FWD_SIDE, 0, 1)
            time.sleep(5); stop()
        elif cmd == 'd': 
            set_motor(FAST_SPEED_FWD_SIDE, FAST_SPEED_REV_SIDE, 1, 0)
            time.sleep(5); stop()
            
        elif cmd == 't':
            try:
                val = input("   Angle? (-360 to 360): ")
                turn_precise(int(val))
            except ValueError: pass

        elif cmd == 'cal':
            print("   1=Prec (75/65), 2=Fast, 3=Spin")
            c = input("   Choice: ")
            if c=='1': 
                # Test RIGHT SPIN (360) using Precision speeds
                # This tests L=75, R=65
                set_motor(PRECISION_SPEED_FWD_SIDE, PRECISION_SPEED_REV_SIDE, 1, 0)
                time.sleep(TIME_360_PREC)
                stop()
            elif c=='2': 
                set_motor(FAST_SPEED_FWD_SIDE, FAST_SPEED_REV_SIDE, 1, 0)
                time.sleep(TIME_360_FAST)
                stop()
            elif c=='3': 
                set_motor(SPIN_SPEED_FWD_SIDE, SPIN_SPEED_REV_SIDE, 1, 0)
                time.sleep(TIME_360_SPIN)
                stop()

        elif cmd == 'q': break
finally:
    stop()
    GPIO.cleanup()

