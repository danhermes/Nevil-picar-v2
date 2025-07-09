#!/usr/bin/env python3
import sys
import os

# Add the local picarx module to path for ROS2 workspace compatibility
current_dir = os.path.dirname(os.path.abspath(__file__))
picarx_path = os.path.join(current_dir, '.')
sys.path.insert(0, picarx_path)

try:
    from .picarx import Picarx
except ImportError:
    # Fallback for direct execution
    from picarx import Picarx

from time import sleep
import readchar
import logging
import datetime

manual = '''
--------------- Picar-X Calibration Helper -----------------1wwds

    [1]: direction servo            [W/D]: increase servo angle
    [2]: camera pan servo           [S/A]: decrease servo angle
    [3]: camera tilt servo          [R]: servos test

    [4]: left motor                 [Q]: change motor direction
    [5]: right motor                [E]: motors run/stop

    [SPACE]: confirm calibration                [Crtl+C]: quit
                                      
'''

# Configure logging with file rotation and multiple outputs
import os
from logging.handlers import RotatingFileHandler

# Ensure log directory exists in project structure
# __file__ is: src/nevil_navigation/nevil_navigation/calibration.py
# We need to go up 3 levels to get to project root
current_file = os.path.abspath(__file__)
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(current_file))))
log_dir = os.path.join(project_root, 'logs')
os.makedirs(log_dir, exist_ok=True)

# Configure main logger
logger = logging.getLogger('servo_calibration')
logger.setLevel(logging.DEBUG)

# Create formatters
detailed_formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - [%(funcName)s:%(lineno)d] - %(message)s'
)
simple_formatter = logging.Formatter(
    '%(asctime)s - %(levelname)s - %(message)s'
)

# Main log file with rotation (10MB max, keep 3 backups)
main_handler = RotatingFileHandler(
    os.path.join(log_dir, 'servo_calibration.log'),
    maxBytes=10*1024*1024,
    backupCount=3
)
main_handler.setLevel(logging.INFO)
main_handler.setFormatter(detailed_formatter)

# Debug log file for detailed troubleshooting
debug_handler = RotatingFileHandler(
    os.path.join(log_dir, 'servo_calibration_debug.log'),
    maxBytes=10*1024*1024,
    backupCount=2
)
debug_handler.setLevel(logging.DEBUG)
debug_handler.setFormatter(detailed_formatter)

# Console output (simplified format)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_handler.setFormatter(simple_formatter)

# Add handlers to logger
logger.addHandler(main_handler)
logger.addHandler(debug_handler)
logger.addHandler(console_handler)

# Prevent duplicate logs
logger.propagate = False

# Global variables - will be initialized in main()
px = None
px_power = 30
servo_num = 0
motor_num = 0
servo_names = ['direction servo', 'camera pan servo', 'camera tilt servo']
motor_names = ['left motor', 'right motor']
servos_cali = []
motors_cali = []
servos_offset = []
motors_offset = []

def servos_test():
    px.set_dir_servo_angle(-30)
    sleep(0.5)
    px.set_dir_servo_angle(30)
    sleep(0.5)
    px.set_dir_servo_angle(0)
    sleep(0.5)
    px.set_cam_pan_angle(-30)
    sleep(0.5)
    px.set_cam_pan_angle(30)
    sleep(0.5)
    px.set_cam_pan_angle(0)
    sleep(0.5)
    px.set_cam_tilt_angle(-30)
    sleep(0.5)
    px.set_cam_tilt_angle(30)
    sleep(0.5)
    px.set_cam_tilt_angle(0)
    sleep(0.5)

def servos_move(servo_num, value):
    servo_name = servo_names[servo_num]
    logger.debug(f"Moving {servo_name} to angle {value}")
    
    if servo_num == 0:
        px.set_dir_servo_angle(value)
    elif servo_num == 1:
        px.set_cam_pan_angle(value)
    elif servo_num == 2:
        px.set_cam_tilt_angle(value)
    sleep(0.2)

def set_servos_offset(servo_num, value):
    servo_name = servo_names[servo_num]
    old_value = servos_offset[servo_num]
    logger.debug(f"Setting {servo_name} offset: {old_value} -> {value}")
    
    if servo_num == 0:
        px.dir_cali_val = value
    elif servo_num == 1:
        px.cam_pan_cali_val = value
    elif servo_num == 2:
        px.cam_tilt_cali_val  = value

def servos_reset():
    for i in range(3):
        servos_move(i,0)

def show_info():
    print("\033[H\033[J", end='')  # clear terminal windows
    print(manual)
    print('[ %s ] [ %s ]'%(servo_names[servo_num], motor_names[motor_num])) 
    print('offset: %s, %s'%(servos_offset, motors_offset))


def cali_helper():
    global px, servo_num, motor_num
    global servos_cali, motors_cali, servos_offset, motors_offset
    
    # Initialize Picarx only when actually running calibration
    px = Picarx()
    
    # Initialize calibration values from the hardware
    servos_cali = [px.dir_cali_val, px.cam_pan_cali_val, px.cam_tilt_cali_val]
    motors_cali = px.cali_dir_value
    servos_offset = list.copy(servos_cali)
    motors_offset = list.copy(motors_cali)
    
    # Log initial calibration state
    logger.info("=== Servo Calibration Session Started ===")
    logger.info(f"Initial servo calibration values: dir={servos_cali[0]}, pan={servos_cali[1]}, tilt={servos_cali[2]}")
    logger.info(f"Initial motor calibration values: {motors_cali}")
    logger.info(f"Configuration source: {px.CONFIG}")
    logger.debug(f"picarx_dir_servo config key location: fileDB.get('picarx_dir_servo')")
    
    motor_run = False
    step = 0.4
    # step = (180 / 2000) * (20000 / 4095)  # actual precision of steering gear

    # reset
    servos_reset()
    # show_info
    show_info()

    # key control
    while True:
        # readkey
        key = readchar.readkey()
        key = key.lower()
        # select the servo 
        if key in ('123'):
            servo_num = int(key)-1
            show_info()
        if key in ('45'):
            motor_num = int(key)-4
            show_info()
        # servos move
        elif key == 'r':
            servos_test()
        elif key == 'w' or key == 'd':
            old_offset = servos_offset[servo_num]
            servos_offset[servo_num] += step
            #if servos_offset[servo_num] > 20:
            #    servos_offset[servo_num] =20
            servos_offset[servo_num] = round(servos_offset[servo_num], 2)
            logger.info(f"Increased {servo_names[servo_num]} offset: {old_offset} -> {servos_offset[servo_num]} (step: +{step})")
            show_info()
            set_servos_offset(servo_num, servos_offset[servo_num])
            servos_move(servo_num, 0)
        elif key == 's' or key == 'a':
            old_offset = servos_offset[servo_num]
            servos_offset[servo_num] -= step
            if servos_offset[servo_num] < -20:
                logger.warning(f"{servo_names[servo_num]} offset constrained to minimum: {servos_offset[servo_num]} -> -20")
                servos_offset[servo_num] = -20
            servos_offset[servo_num] = round(servos_offset[servo_num], 2)
            logger.info(f"Decreased {servo_names[servo_num]} offset: {old_offset} -> {servos_offset[servo_num]} (step: -{step})")
            show_info()
            set_servos_offset(servo_num, servos_offset[servo_num])
            servos_move(servo_num, 0)
        # motors move
        elif key == 'q': 
            motors_offset[motor_num] = -1 * motors_offset[motor_num]
            px.cali_dir_value = list.copy(motors_offset)
            motor_run = True
            px.forward(px_power)
            show_info()
        elif key == 'e':
            if motor_run == False:
                motor_run = True
                px.forward(px_power)
            else:
                motor_run = False
                px.stop()
        # save
        elif key == readchar.key.SPACE:
            print('Confirm save ?(y/n)')
            while True:
                key = readchar.readkey()
                key = key.lower()
                if key == 'y':
                    logger.info("=== Starting Servo Calibration Save Process ===")
                    
                    # Direction servo calibration
                    old_dir_val = px.dir_cali_val
                    logger.info(f"Direction servo: old_value={old_dir_val}, new_value={servos_offset[0]}")
                    logger.debug(f"Calling px.dir_servo_calibrate({servos_offset[0]}) - saves to 'picarx_dir_servo' config key")
                    try:
                        px.dir_servo_calibrate(servos_offset[0])
                        logger.info(f"Direction servo calibration successful: {old_dir_val} -> {px.dir_cali_val}")
                    except Exception as e:
                        logger.error(f"Direction servo calibration failed: {e}")
                    
                    # Camera pan servo calibration
                    old_pan_val = px.cam_pan_cali_val
                    logger.info(f"Camera pan servo: old_value={old_pan_val}, new_value={servos_offset[1]}")
                    logger.debug(f"Calling px.cam_pan_servo_calibrate({servos_offset[1]}) - saves to 'picarx_cam_pan_servo' config key")
                    try:
                        px.cam_pan_servo_calibrate(servos_offset[1])
                        logger.info(f"Camera pan servo calibration successful: {old_pan_val} -> {px.cam_pan_cali_val}")
                    except Exception as e:
                        logger.error(f"Camera pan servo calibration failed: {e}")
                    
                    # Camera tilt servo calibration
                    old_tilt_val = px.cam_tilt_cali_val
                    logger.info(f"Camera tilt servo: old_value={old_tilt_val}, new_value={servos_offset[2]}")
                    logger.debug(f"Calling px.cam_tilt_servo_calibrate({servos_offset[2]}) - saves to 'picarx_cam_tilt_servo' config key")
                    try:
                        px.cam_tilt_servo_calibrate(servos_offset[2])
                        logger.info(f"Camera tilt servo calibration successful: {old_tilt_val} -> {px.cam_tilt_cali_val}")
                    except Exception as e:
                        logger.error(f"Camera tilt servo calibration failed: {e}")
                    
                    # Motor direction calibration
                    old_motor_val = motors_offset[motor_num]
                    logger.info(f"Motor {motor_num + 1} direction: old_value={old_motor_val}, new_value={motors_offset[motor_num]}")
                    try:
                        px.motor_direction_calibrate(motor_num +1 , motors_offset[motor_num])
                        logger.info(f"Motor {motor_num + 1} direction calibration successful")
                    except Exception as e:
                        logger.error(f"Motor {motor_num + 1} direction calibration failed: {e}")
                    
                    sleep(0.2)
                    
                    # Verify calibration persistence
                    servos_offset = [px.dir_cali_val, px.cam_pan_cali_val, px.cam_tilt_cali_val]
                    logger.info(f"Post-calibration verification: dir={px.dir_cali_val}, pan={px.cam_pan_cali_val}, tilt={px.cam_tilt_cali_val}")
                    logger.info("=== Servo Calibration Save Process Complete ===")
                    
                    show_info()
                    print('The calibration value has been saved.')
                    break
                elif key == 'n':
                    show_info()
                    break   
                sleep(0.01) 

        # quit
        elif key == readchar.key.CTRL_C or key in readchar.key.ESC:
            logger.info("=== Servo Calibration Session Terminated by User ===")
            print('quit')
            break

        sleep(0.01)


def main():
    """Main entry point for ROS2 console script."""
    try:
        cali_helper()
    except KeyboardInterrupt:
        logger.info("=== Servo Calibration Session Interrupted ===")
        print('quit')
    except Exception as e:
        logger.error(f"Servo calibration session failed with exception: {e}")
        print(e)
    finally:
        logger.info("=== Servo Calibration Session Cleanup ===")
        px.stop()
        sleep(0.1)
        logger.info("=== Servo Calibration Session Ended ===")


if __name__ == "__main__":
    main()
