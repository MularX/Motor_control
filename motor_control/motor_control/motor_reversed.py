import RPi.GPIO as GPIO
import curses
import time

# ========== GPIO Setup ==========

# Definicja pinów dla każdego silnika (PWM, DIR)
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
    pwm = GPIO.PWM(pins['PWM'], 10000)  # 10 kHz PWM
    pwm.start(0)
    pwm_instances[motor_id] = pwm

speed = 30  # Początkowa prędkość (0–100%)
current_mode = None  # 'forward', 'backward', 'left', 'right', None

# ========== Funkcje sterujące ==========

def set_motor(motor_id, forward: bool, speed_percent: int):
    # Odwrócona logika: LOW = do przodu, HIGH = wstecz
    dir_state = GPIO.LOW if forward else GPIO.HIGH
    GPIO.output(MOTOR_PINS[motor_id]['DIR'], dir_state)
    pwm_instances[motor_id].ChangeDutyCycle(speed_percent)

def stop_all():
    for motor_id in MOTOR_PINS:
        pwm_instances[motor_id].ChangeDutyCycle(0)
        GPIO.output(MOTOR_PINS[motor_id]['DIR'], GPIO.LOW)  # Neutral logic

def move_forward():
    set_motor(1, True, speed)
    set_motor(2, True, speed)
    set_motor(3, True, speed)
    set_motor(4, True, speed)

def move_backward():
    set_motor(1, False, speed)
    set_motor(2, False, speed)
    set_motor(3, False, speed)
    set_motor(4, False, speed)

def turn_left():
    set_motor(1, False, speed)
    set_motor(2, False, speed)
    set_motor(3, True, speed)
    set_motor(4, True, speed)

def turn_right():
    set_motor(1, True, speed)
    set_motor(2, True, speed)
    set_motor(3, False, speed)
    set_motor(4, False, speed)

# ========== Curses interface ==========

def main(stdscr):
    global speed, current_mode
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()

    stdscr.addstr(0, 0, "Sterowanie silnikami | Strzałki: ruch | +/-: prędkość | ESC: wyjście")

    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:
            move_forward()
            current_mode = 'forward'
            stdscr.addstr(1, 0, "Jazda do przodu        ")
        elif key == curses.KEY_DOWN:
            move_backward()
            current_mode = 'backward'
            stdscr.addstr(1, 0, "Jazda do tyłu          ")
        elif key == curses.KEY_LEFT:
            turn_left()
            current_mode = 'left'
            stdscr.addstr(1, 0, "Skręt w lewo           ")
        elif key == curses.KEY_RIGHT:
            turn_right()
            current_mode = 'right'
            stdscr.addstr(1, 0, "Skręt w prawo          ")
        elif key == ord('+'):
            speed = min(100, speed + 10)
            stdscr.addstr(2, 0, f"Prędkość: {speed}%       ")
            if current_mode == 'forward':
                move_forward()
            elif current_mode == 'backward':
                move_backward()
            elif current_mode == 'left':
                turn_left()
            elif current_mode == 'right':
                turn_right()
        elif key == ord('-'):
            speed = max(0, speed - 10)
            stdscr.addstr(2, 0, f"Prędkość: {speed}%       ")
            if current_mode == 'forward':
                move_forward()
            elif current_mode == 'backward':
                move_backward()
            elif current_mode == 'left':
                turn_left()
            elif current_mode == 'right':
                turn_right()
        elif key == 27:  # ESC key
            break
        else:
            stop_all()
            current_mode = None
            stdscr.addstr(1, 0, "Silniki zatrzymane     ")

        stdscr.refresh()
        time.sleep(0.05)

# ========== Cleanup ==========

try:
    curses.wrapper(main)
finally:
    stop_all()
    for pwm in pwm_instances.values():
        pwm.stop()
    GPIO.cleanup()
    print("Zakończono program.")

