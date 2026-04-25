import RPi.GPIO as GPIO
import time

# ==========================================
# --- USER CALIBRATION (CRITICAL) ---
# ==========================================
# Measure this! Run the 'cal' command at Speed 100 for 1 second.
# Enter the distance it traveled here (in cm).
TOP_SPEED_CM_PER_SEC = 67   

# ==========================================
# --- PINS & SETUP ---
ENA, IN1, IN2 = 25, 24, 23
IN3, IN4, ENB = 27, 17, 22
GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
GPIO.setup([ENA, IN1, IN2, IN3, IN4, ENB], GPIO.OUT)
pwm_left = GPIO.PWM(ENA, 100); pwm_right = GPIO.PWM(ENB, 100)
pwm_left.start(0); pwm_right.start(0)

def set_motor(speed, dir_l, dir_r):
    # Simple straight drive (Left and Right same speed)
    GPIO.output(IN1, GPIO.HIGH if dir_l else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW if dir_l else GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    GPIO.output(IN3, GPIO.HIGH if dir_r else GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW if dir_r else GPIO.HIGH)
    pwm_right.ChangeDutyCycle(speed)

def stop():
    pwm_left.ChangeDutyCycle(0); pwm_right.ChangeDutyCycle(0)
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

def move_calculated(target_speed, target_distance_cm):
    """
    Calculates time based on Distance / Speed
    """
    # 1. Estimate physical speed (cm/s) at this Duty Cycle
    # Formula: (Current_Duty / 100) * Top_Speed
    estimated_speed_cm_s = (target_speed / 100.0) * TOP_SPEED_CM_PER_SEC
    
    # Avoid divide by zero
    if estimated_speed_cm_s < 1: estimated_speed_cm_s = 1
    
    # 2. Calculate Time
    duration = target_distance_cm / estimated_speed_cm_s
    
    print(f"--- CALCULATION ---")
    print(f"Target:   {target_distance_cm} cm")
    print(f"Speed:    {target_speed}% (~{estimated_speed_cm_s:.1f} cm/s)")
    print(f"Duration: {duration:.2f} seconds")
    
    # 3. Move
    set_motor(target_speed, 1, 1) # Forward
    time.sleep(duration)
    stop()

# --- MAIN MENU ---
print("\n=== DISTANCE CALCULATOR ===")
print(f"Calibrated Top Speed: {TOP_SPEED_CM_PER_SEC} cm/s")

try:
    while True:
        print("\nSelect Mode:")
        print("1. Set Speed -> Input Distance")
        print("2. Set Distance -> Input Speed")
        print("cal. Measure Top Speed")
        print("q. Quit")
        cmd = input("> ").lower().strip()
        
        if cmd == '1':
            # MODE 1: Fixed Speed, User picks Distance
            spd = float(input("   Enter Speed (0-100): "))
            dist = float(input("   Enter Distance (cm): "))
            move_calculated(spd, dist)
            
        elif cmd == '2':
            # MODE 2: Fixed Distance, User picks Speed
            dist = float(input("   Enter Distance (cm): "))
            spd = float(input("   Enter Speed (0-100): "))
            move_calculated(spd, dist)
            
        elif cmd == 'cal':
            print("   Running at Speed 100 for 1.0 second...")
            print("   Measure the distance and update TOP_SPEED_CM_PER_SEC.")
            time.sleep(1)
            set_motor(100, 1, 1)
            time.sleep(1.0)
            stop()
            
        elif cmd == 'q': break

finally:
    stop()
    GPIO.cleanup()
