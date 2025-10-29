import RPi.GPIO as GPIO
import curses
import time

# ========== GPIO Setup ==========

MOTOR_PINS = {
    1: {'PWM': 18, 'DIR': 17},
    2: {'PWM': 23, 'DIR': 24},
    3: {'PWM': 12, 'DIR': 16},
    4: {'PWM': 21, 'DIR': 5},
}

GPIO.setmode(GPIO.BCM)
pwm_instances = {}

# Inicjalizacja pinów
for motor_id, pins in MOTOR_PINS.items():
    GPIO.setup(pins['PWM'], GPIO.OUT)
    GPIO.setup(pins['DIR'], GPIO.OUT)
    pwm = GPIO.PWM(pins['PWM'], 10000)  # 10 kHz
    pwm.start(0)
    pwm_instances[motor_id] = pwm

speed = 50  # Początkowa prędkość (0-100)

# ========== Sterowanie silnikami ==========

def set_motor(motor_id, direction: bool, speed_percent: int):
    GPIO.output(MOTOR_PINS[motor_id]['DIR'], GPIO.HIGH if direction else GPIO.LOW)
    pwm_instances[motor_id].ChangeDutyCycle(speed_percent)

def stop_all():
    for motor_id in MOTOR_PINS:
        pwm_instances[motor_id].ChangeDutyCycle(100)

def move_forward():
    set_motor(1, True, speed)
    set_motor(2, False, speed)
    set_motor(3, False, speed)
    set_motor(4, True, speed)

def move_backward():
    set_motor(1, False, speed)
    set_motor(2, True, speed)
    set_motor(3, True, speed)
    set_motor(4, False, speed)

def turn_left():
    set_motor(1, True, speed)
    set_motor(2, True, speed)
    set_motor(3, True, speed)
    set_motor(4, True, speed)

def turn_right():
    set_motor(1, False, speed)
    set_motor(2, False, speed)
    set_motor(3, False, speed)
    set_motor(4, False, speed)
    
def arc_NE():
    set_motor(1, True, 0)
    set_motor(2, False, 99)
    set_motor(3, False, 99)
    set_motor(4, True, 0)
    
def arc_NW():
    set_motor(1, True, 90)
    set_motor(2, False, 0)
    set_motor(3, False, 0)
    set_motor(4, True, 90)
    
def arc_SW():
    set_motor(1, False, 90)
    set_motor(2, True, 0)
    set_motor(3, True, 0)
    set_motor(4, False, 90)
    
def arc_SE():
    set_motor(1, False, 0)
    set_motor(2, True, 90)
    set_motor(3, True, 90)
    set_motor(4, False, 0)

def main(stdscr):
    global speed
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()

    stdscr.addstr(0, 0, "Strzałki: sterowanie | ESC: wyjście")

    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:
            move_forward()
            stdscr.addstr(1, 0, "Jazda do przodu     ")
        elif key == curses.KEY_DOWN:
            move_backward()
            stdscr.addstr(1, 0, "Jazda do tyłu       ")
        elif key == curses.KEY_LEFT:
            turn_left()
            stdscr.addstr(1, 0, "Skręt w lewo        ")
        elif key == curses.KEY_RIGHT:
            turn_right()
            stdscr.addstr(1, 0, "Skręt w prawo       ")
        elif key == ord('+'):
            speed = min(100, speed + 10)
            stdscr.addstr(2, 0, f"Prędkość: {speed}%     ")
        elif key == ord('-'):
            speed = max(0, speed - 10)
            stdscr.addstr(2, 0, f"Prędkość: {speed}%     ")
        elif key == ord('e'):
            arc_NE()
            stdscr.addstr(1, 0, "arc NE       ")
        elif key == ord('r'):
            arc_NW()
            stdscr.addstr(1, 0, "arc NW       ")
        elif key == ord('f'):
            arc_SW()
            stdscr.addstr(1, 0, "arc SW       ")
        elif key == ord('d'):
            arc_SE()
            stdscr.addstr(1, 0, "arc SE     ")
        elif key == 27:  # ESC
            break
        else:
            stop_all()
            stdscr.addstr(1, 0, "Silniki zatrzymane  ")

        stdscr.refresh()
        time.sleep(0.05)

try:
    curses.wrapper(main)

finally:
    stop_all()
    for pwm in pwm_instances.values():
        pwm.stop()
    GPIO.cleanup()
    print("Zakończono program.")
