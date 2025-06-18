from time import sleep
from helpers.utils import gray_print
from vilib import Vilib
import time

# Actions: forward, backward, left, right, stop, twist left, twist right, come here, shake head, 
#    nod, wave hands, resist, act cute, rub hands, think, twist body, celebrate, depressed, keep think
#
# Sounds: honk, start engine

def with_obstacle_check(func):
    """Decorator to add obstacle checking to movement functions
    Uses move_forward's logic: 
    - SafeDistance: go straight
    - DangerDistance to SafeDistance: turn to avoid
    - Below DangerDistance: back up
    """
    def wrapper(car, *args, **kwargs):
        def check_distance():
            distance = car.get_distance()
            if distance >= car.SafeDistance:
                #car.set_dir_servo_angle(0)
                return "safe"
            elif distance >= car.DangerDistance:
                car.set_dir_servo_angle(30)
                return "caution"
            else:
                car.set_dir_servo_angle(-30)
                move_backward_this_way(car, 10, car.speed)
                sleep(0.5)
                return "danger"
            
        return func(car, *args, check_distance=check_distance, **kwargs)
    return wrapper

@with_obstacle_check
def move_forward_this_way(car, distance_cm, speed=None, check_distance=None):
    """Move forward a specific distance at given speed"""
    distance_cm = distance_cm * 3 #calibrate distance
    if speed is None:
        speed = car.speed
    gray_print(f"Starting forward movement: distance={distance_cm}cm, speed={speed}")
    
    SPEED_TO_CM_PER_SEC = 0.7  # Needs calibration
    move_time = distance_cm / (speed * SPEED_TO_CM_PER_SEC)
    gray_print(f"Calculated move time: {move_time:.2f} seconds")
    elapsed_time = 0
    
    while elapsed_time < move_time:
        status = check_distance()
        if status == "danger":
            gray_print("Obstacle too close! Backing up.")
            return
        elif status == "caution":
            gray_print("Obstacle detected! Adjusting course.")
            
        car.forward(speed)
        sleep(0.1)
        elapsed_time += 0.1
        if elapsed_time % 1 < 0.1:
            gray_print(f"Moving... elapsed={elapsed_time:.1f}s")
    
    gray_print("Movement complete, stopping")
    car.stop()

# def move_forward(car):
#     distance = car.get_distance()
#     if distance >= car.SafeDistance:
#         car.set_dir_servo_angle(0)
#         car.forward(car.speed)
#     elif distance >= car.DangerDistance:
#         car.set_dir_servo_angle(30)
#         car.forward(car.speed)
#         sleep(0.1)
#     else:
#         car.set_dir_servo_angle(-30)
#         car.backward(car.speed)

def move_backward_this_way(car, distance_cm, speed=None):
    """Move backward a specific distance at given speed"""
    if speed is None:
        speed = car.speed
    gray_print(f"Starting backward movement: distance={distance_cm}cm, speed={speed}")
    
    SPEED_TO_CM_PER_SEC = 0.7  # Needs calibration
    move_time = distance_cm / (speed * SPEED_TO_CM_PER_SEC)
    gray_print(f"Calculated move time: {move_time:.2f} seconds")
    elapsed_time = 0
    
    while elapsed_time < move_time:
        car.backward(speed)
        sleep(0.1)
        elapsed_time += 0.1
        if elapsed_time % 1 < 0.1:
            gray_print(f"Moving... elapsed={elapsed_time:.1f}s")
    
    gray_print("Movement complete, stopping")
    car.stop()

def turn_left(car):
    gray_print("Starting left turn sequence")
    gray_print("Setting wheel angle to -30°")
    car.set_dir_servo_angle(-30)
    gray_print("Moving forward first segment")
    move_forward_this_way(car, 20)
    gray_print("Straightening wheels")
    car.set_dir_servo_angle(0)
    gray_print("Moving forward second segment")
    move_forward_this_way(car, 20)
    gray_print("Left turn complete")

def turn_right(car):
    gray_print("Starting right turn sequence")
    gray_print("Setting wheel angle to 30°")
    car.set_dir_servo_angle(30)
    gray_print("Moving forward first segment")
    move_forward_this_way(car, 20)
    gray_print("Straightening wheels")
    car.set_dir_servo_angle(0)
    gray_print("Moving forward second segment")
    move_forward_this_way(car, 20)
    gray_print("Right turn complete")

def stop(car):
    car.stop()

def turn_left_in_place(car):
    car.set_dir_servo_angle(-30)

def turn_right_in_place(car):
    car.set_dir_servo_angle(30)

# @with_obstacle_check
# def come_here(car, check_distance=None):
#     car.Vilib.face_detect_switch(True)
#     speed = 15
#     dir_angle = 0
#     x_angle = 0
#     y_angle = 0
    
#     while True:
#         status = check_distance()
#         if status == "danger":
#             gray_print("Obstacle too close! Backing up.")
#             break
#         elif status == "caution":
#             gray_print("Obstacle detected! Adjusting course.")
            
#         if car.Vilib.detect_obj_parameter['face'] != 0:
#             coordinate_x = Vilib.detect_obj_parameter['face_x']
#             coordinate_y = Vilib.detect_obj_parameter['face_y']
            
#             x_angle += (coordinate_x*10/640)-5
#             x_angle = clamp_number(x_angle,-35,35)
#             car.set_cam_pan_angle(x_angle)

#             y_angle -= (coordinate_y*10/480)-5
#             y_angle = clamp_number(y_angle,-35,35)
#             car.set_cam_tilt_angle(y_angle)

#             if dir_angle > x_angle:
#                 dir_angle -= 1
#             elif dir_angle < x_angle:
#                 dir_angle += 1
#             car.set_dir_servo_angle(x_angle)
#             move_forward_this_way(car,10,40)
#             sleep(0.05)
#         else:
#             move_forward_this_way(car,5,5)
#             Vilib.face_detect_switch(False)
#             sleep(0.05)

def clamp_number(num,a,b):
  return max(min(num, max(a, b)), min(a, b))

def wave_hands(car):
    car.reset()
    car.set_cam_tilt_angle(20)
    for _ in range(2):
        car.set_dir_servo_angle(-25)
        sleep(.1)
        # car.set_dir_servo_angle(0)
        # sleep(.1)
        car.set_dir_servo_angle(25)
        sleep(.1)
    car.set_dir_servo_angle(0)

def resist(car):
    car.reset()
    car.set_cam_tilt_angle(10)
    for _ in range(3):
        car.set_dir_servo_angle(-15)
        car.set_cam_pan_angle(15)
        sleep(.1)
        car.set_dir_servo_angle(15)
        car.set_cam_pan_angle(-15)
        sleep(.1)
    car.stop()
    car.set_dir_servo_angle(0)
    car.set_cam_pan_angle(0)

def act_cute(car):
    car.reset()
    car.set_cam_tilt_angle(-20)
    for i in range(15):
        car.forward(5)
        sleep(0.02)
        car.backward(5)
        sleep(0.02)
    car.set_cam_tilt_angle(0)
    car.stop()

def rub_hands(car):
    car.reset()
    for i in range(5):
        car.set_dir_servo_angle(-6)
        sleep(.5)
        car.set_dir_servo_angle(6)
        sleep(.5)
    car.reset()

def think(car):
    car.reset()
    for i in range(11):
        car.set_cam_pan_angle(i*3)
        car.set_cam_tilt_angle(-i*2)
        car.set_dir_servo_angle(i*2)
        sleep(.05)
    sleep(1)
    car.set_cam_pan_angle(15)
    car.set_cam_tilt_angle(-10)
    car.set_dir_servo_angle(10)
    sleep(.1)
    car.reset()

def keep_think(car):
    car.reset()
    for i in range(11):
        car.set_cam_pan_angle(i*3)
        car.set_cam_tilt_angle(-i*2)
        car.set_dir_servo_angle(i*2)
        sleep(.05)

def shake_head(car):
    car.stop()
    car.set_cam_pan_angle(0)
    car.set_cam_pan_angle(60)
    sleep(.2)
    car.set_cam_pan_angle(-50)
    sleep(.1)
    car.set_cam_pan_angle(40)
    sleep(.1)
    car.set_cam_pan_angle(-30)
    sleep(.1)
    car.set_cam_pan_angle(20)
    sleep(.1)
    car.set_cam_pan_angle(-10)
    sleep(.1)
    car.set_cam_pan_angle(10)
    sleep(.1)
    car.set_cam_pan_angle(-5)
    sleep(.1)
    car.set_cam_pan_angle(0)

def nod(car):
    car.reset()
    car.set_cam_tilt_angle(0)
    car.set_cam_tilt_angle(5)
    sleep(.1)
    car.set_cam_tilt_angle(-30)
    sleep(.1)
    car.set_cam_tilt_angle(5)
    sleep(.1)
    car.set_cam_tilt_angle(-30)
    sleep(.1)
    car.set_cam_tilt_angle(0)


def depressed(car):
    car.reset()
    car.set_cam_tilt_angle(0)
    car.set_cam_tilt_angle(20)
    sleep(.22)
    car.set_cam_tilt_angle(-22)
    sleep(.1)
    car.set_cam_tilt_angle(10)
    sleep(.1)
    car.set_cam_tilt_angle(-22)
    sleep(.1)
    car.set_cam_tilt_angle(0)
    sleep(.1)
    car.set_cam_tilt_angle(-22)
    sleep(.1)
    car.set_cam_tilt_angle(-10)
    sleep(.1)
    car.set_cam_tilt_angle(-22)
    sleep(.1)
    car.set_cam_tilt_angle(-15)
    sleep(.1)
    car.set_cam_tilt_angle(-22)
    sleep(.1)
    car.set_cam_tilt_angle(-19)
    sleep(.1)
    car.set_cam_tilt_angle(-22)
    sleep(.1)

    sleep(1.5)
    car.reset()

def twist_body(car):
    car.reset()
    for i in range(3):
        car.set_motor_speed(1, 20)
        car.set_motor_speed(2, 20)
        car.set_cam_pan_angle(-20)
        car.set_dir_servo_angle(-10)
        sleep(.1)
        car.set_motor_speed(1, 0)
        car.set_motor_speed(2, 0)
        car.set_cam_pan_angle(0)
        car.set_dir_servo_angle(0)
        sleep(.1)
        car.set_motor_speed(1, -20)
        car.set_motor_speed(2, -20)
        car.set_cam_pan_angle(20)
        car.set_dir_servo_angle(10)
        sleep(.1)
        car.set_motor_speed(1, 0)
        car.set_motor_speed(2, 0)
        car.set_cam_pan_angle(0)
        car.set_dir_servo_angle(0)

        sleep(.1)


def celebrate(car):
    car.reset()
    car.set_cam_tilt_angle(20)

    car.set_dir_servo_angle(30)
    car.set_cam_pan_angle(60)
    sleep(.3)
    car.set_dir_servo_angle(10)
    car.set_cam_pan_angle(30)
    sleep(.1)
    car.set_dir_servo_angle(30)
    car.set_cam_pan_angle(60)
    sleep(.3)
    car.set_dir_servo_angle(0)
    car.set_cam_pan_angle(0)
    sleep(.2)

    car.set_dir_servo_angle(-30)
    car.set_cam_pan_angle(-60)
    sleep(.3)
    car.set_dir_servo_angle(-10)
    car.set_cam_pan_angle(-30)
    sleep(.1)
    car.set_dir_servo_angle(-30)
    car.set_cam_pan_angle(-60)
    sleep(.3)
    car.set_dir_servo_angle(0)
    car.set_cam_pan_angle(0)
    sleep(.2)

def honk(car):
    print(f"Honk called with car object: {car}, has music: {hasattr(car, 'music')}")
    try:
        if hasattr(car, 'music'):
            print(f"Car music object: {car.music}")
            car.music.sound_play("../sounds/car-double-horn.wav", 100)
            while car.music.pygame.mixer.music.get_busy():
                time.sleep(0.1)
        else:
            gray_print("Warning: Car does not have audio capabilities")
    except Exception as e:
        gray_print(f"Error playing honk sound: {e}")

def start_engine(car):
    print(f"Start engine called with car object: {car}, has music: {hasattr(car, 'music')}")
    try:
        if hasattr(car, 'music'):
            print(f"Car music object: {car.music}")
            car.music.sound_play("../sounds/car-start-engine.wav", 50)
            while car.music.pygame.mixer.music.get_busy():
                time.sleep(0.1)
        else:
            gray_print("Warning: Car does not have audio capabilities")
    except Exception as e:
        gray_print(f"Error playing engine sound: {e}")

# Define dictionaries after all functions are defined
actions_dict = {
    "forward": move_forward_this_way,
    "backward": move_backward_this_way,
    "left": turn_left,
    "right": turn_right,
    "stop": stop,
    "twist left": turn_left_in_place,
    "twist right": turn_right_in_place,
    #"come here": come_here,
    "shake head": shake_head,
    "nod": nod,
    "wave hands": wave_hands,
    "resist": resist,
    "act cute": act_cute,
    "rub hands": rub_hands,
    "think": think,
    "twist body": twist_body,
    "celebrate": celebrate,
    "depressed": depressed,
    "think": think,
    "keep think": keep_think,
    "honk": honk,
    "start engine": start_engine
}










