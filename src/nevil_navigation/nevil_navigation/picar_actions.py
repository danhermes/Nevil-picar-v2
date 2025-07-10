import sys
import os
from time import sleep
import time

# Actions: forward, backward, left, right, stop, twist left, twist right, come here, shake head,
#    nod, wave hands, resist, act cute, rub hands, think, twist body, celebrate, depressed, keep think
#
# Sounds: honk, start engine
 
class PicarActions:
    def __init__(self, car_instance=None, logger=None):
        self.logger = logger
        if self.logger:
            self.logger.info(f"Starting PicarActions().")
        
        # Use provided car instance or initialize our own
        if car_instance is not None:
            self.car = car_instance
            self.speed = 30
            if self.logger:
                self.logger.info("Using provided car instance")
            # Initialize wheels to straight position
            self.initialize_servos()
            # Note: servos_test() can be called manually when needed
        else:
            if self.logger:
                self.logger.error("No PicarX available.")
    #         # Initialize car hardware like v1.0
    #         self.car = None
    #         self.speed = 30
    #         self.initialize_car()
    
    # def initialize_car(self):
    #     """Initialize PiCar hardware."""
    #     try:
    #         from .picarx import Picarx
    #         from robot_hat import reset_mcu
            
    #         # Reset MCU
    #         reset_mcu()
    #         time.sleep(0.1)
            
    #         # Create car instance
    #         self.car = Picarx()
    #         self.car.SafeDistance = 30
    #         self.car.DangerDistance = 15
    #         self.car.speed = 30
    #         self.speed = 30
            
    #         time.sleep(1)
    #         self.logger.info("PiCar hardware initialized successfully")
            
    #     except Exception as e:
    #         self.car = None
    #         self.logger.warn(f"Failed to initialize PiCar hardware: {e}")
    #         self.logger.info("Running in simulation mode without hardware")

    def initialize_servos(self):
        """Initialize servos to straight position (0 degrees)"""
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot initialize servos: No car instance available")
            return
       
        if self.logger:
            self.logger.info("Initializing servos to 0 degrees (straight)")

        self.stop() #stop motors
        self.car.set_dir_servo_angle(0) # steering axle
        self.car.set_cam_pan_angle(0)   # head r/l
        self.car.set_cam_tilt_angle(0)  # head u/d
        sleep(0.2)

        # Final verification
        if self.logger:
            self.logger.info("Servo initialization complete - wheels should now be straight")


    def servos_test(self):
        self.car.set_dir_servo_angle(-30)
        sleep(0.5)
        self.car.set_dir_servo_angle(30)
        sleep(0.5)
        self.car.set_dir_servo_angle(0)
        sleep(0.5)
        self.car.set_cam_pan_angle(-30)
        sleep(0.5)
        self.car.set_cam_pan_angle(30)
        sleep(0.5)
        self.car.set_cam_pan_angle(0)
        sleep(0.5)
        self.car.set_cam_tilt_angle(-30)
        sleep(0.5)
        self.car.set_cam_tilt_angle(30)
        sleep(0.5)
        self.car.set_cam_tilt_angle(0)
        sleep(0.5)
        self.car.reset()

    # Removed obstacle check decorator - will be reimplemented when hardware integration is added

    def move_forward_this_way(self, distance_cm=20, speed=None):
        """Move forward a specific distance at given speed"""
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot move forward: No car instance available")
            return
            
        distance_cm = distance_cm * 3 #calibrate distance
        if speed is None:
            speed = self.speed
        if self.logger:
            self.logger.info(f"Starting forward movement: distance={distance_cm}cm, speed={speed}")
        
        SPEED_TO_CM_PER_SEC = 0.7  # Needs calibration
        move_time = distance_cm / (speed * SPEED_TO_CM_PER_SEC)
        if self.logger:
            self.logger.info(f"Calculated move time: {move_time:.2f} seconds")
        elapsed_time = 0
        
        while elapsed_time < move_time:
            self.car.forward(speed)
            sleep(0.1)
            elapsed_time += 0.1
            if elapsed_time % 1 < 0.1:
                if self.logger:
                    self.logger.info(f"Moving... elapsed={elapsed_time:.1f}s")
        
        if self.logger:
            self.logger.info("Movement complete, stopping")
        self.car.stop()

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

    def move_backward_this_way(self, distance_cm=20, speed=None):
        """Move backward a specific distance at given speed"""
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot move backward: No car instance available")
            return
            
        if speed is None:
            speed = self.speed
        if self.logger:
            self.logger.info(f"Starting backward movement: distance={distance_cm}cm, speed={speed}")
        
        SPEED_TO_CM_PER_SEC = 0.7  # Needs calibration
        move_time = distance_cm / (speed * SPEED_TO_CM_PER_SEC)
        if self.logger:
            self.logger.info(f"Calculated move time: {move_time:.2f} seconds")
        elapsed_time = 0
        
        while elapsed_time < move_time:
            self.car.backward(speed)
            sleep(0.1)
            elapsed_time += 0.1
            if elapsed_time % 1 < 0.1:
                if self.logger:
                    self.logger.info(f"Moving... elapsed={elapsed_time:.1f}s")
        
        if self.logger:
            self.logger.info("Movement complete, stopping")
        self.car.stop()

    def turn_left(self):
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot turn left: No car instance available")
            return
        if self.logger:
            self.logger.info("Starting left turn sequence")
        self.car.set_dir_servo_angle(-30)
        if self.logger:
            self.logger.info("Setting wheel angle to -30°")
        self.move_forward_this_way(20)
        self.car.set_dir_servo_angle(0)
        if self.logger:
            self.logger.info("Straightening wheels")
        self.move_forward_this_way(20)
        if self.logger:
            self.logger.info("Left turn complete")

    def turn_right(self):
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot turn right: No car instance available")
            return
        if self.logger:
            self.logger.info("Starting right turn sequence")
        self.car.set_dir_servo_angle(30)
        if self.logger:
            self.logger.info("Setting wheel angle to 30°")
        self.move_forward_this_way(20)
        self.car.set_dir_servo_angle(0)
        if self.logger:
            self.logger.info("Straightening wheels")
        self.move_forward_this_way(20)
        if self.logger:
            self.logger.info("Right turn complete")

    def stop(self):
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot stop: No car instance available")
            return
        if self.logger:
            self.logger.info("Stopping robot")
        self.car.stop()

    def turn_left_in_place(self):
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot turn left in place: No car instance available")
            return
        if self.logger:
            self.logger.info("Turning left in place")
        self.car.set_dir_servo_angle(-30)
        sleep(0.5)  # Allow time for the turn
        self.car.set_dir_servo_angle(0)  # Reset to straight
        if self.logger:
            self.logger.info("Left in-place turn complete, wheels straightened")

    def turn_right_in_place(self):
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot turn right in place: No car instance available")
            return
        if self.logger:
            self.logger.info("Turning right in place")
        self.car.set_dir_servo_angle(30)
        sleep(0.5)  # Allow time for the turn
        self.car.set_dir_servo_angle(0)  # Reset to straight
        if self.logger:
            self.logger.info("Right in-place turn complete, wheels straightened")

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
    #             self.logger.info("Obstacle too close! Backing up.")
    #             break
    #         elif status == "caution":
    #             self.logger.info("Obstacle detected! Adjusting course.")
                
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

    def clamp_number(self, num, a, b):
        return max(min(num, max(a, b)), min(a, b))

    def wave_hands(self):
        if self.logger:
            self.logger.info("Waving hands")
        self.car.reset()
        self.car.set_cam_tilt_angle(20)
        for _ in range(2):
            self.car.set_dir_servo_angle(-25)
            sleep(.1)
            self.car.set_dir_servo_angle(25)
            sleep(.1)
        self.car.set_dir_servo_angle(0)

    def resist(self):
        if self.logger:
            self.logger.info("Resisting")
        self.car.reset()
        self.car.set_cam_tilt_angle(10)
        for _ in range(3):
            self.car.set_dir_servo_angle(-15)
            self.car.set_cam_pan_angle(15)
            sleep(.1)
            self.car.set_dir_servo_angle(15)
            self.car.set_cam_pan_angle(-15)
            sleep(.1)
        self.car.stop()
        self.car.set_dir_servo_angle(0)
        self.car.set_cam_pan_angle(0)

    def act_cute(self):
        if self.logger:
            self.logger.info("Acting cute")
        self.car.reset()
        self.car.set_cam_tilt_angle(-20)
        for i in range(15):
            self.car.forward(5)
            sleep(0.02)
            self.car.backward(5)
            sleep(0.02)
        self.car.set_cam_tilt_angle(0)
        self.car.stop()

    def rub_hands(self):
        if self.logger:
            self.logger.info("Rubbing hands")
        self.car.reset()
        for i in range(5):
            self.car.set_dir_servo_angle(-6)
            sleep(.5)
            self.car.set_dir_servo_angle(6)
            sleep(.5)
        self.car.reset()

    def think(self):
        if self.logger:
            self.logger.info("Thinking")
        self.car.reset()
        for i in range(11):
            self.car.set_cam_pan_angle(i*3)
            self.car.set_cam_tilt_angle(-i*2)
            self.car.set_dir_servo_angle(i*2)
            sleep(.05)
        sleep(1)
        self.car.set_cam_pan_angle(15)
        self.car.set_cam_tilt_angle(-10)
        self.car.set_dir_servo_angle(10)
        sleep(.1)
        self.car.reset()

    def keep_think(self):
        if self.logger:
            self.logger.info("Keep thinking")
        self.car.reset()
        for i in range(11):
            self.car.set_cam_pan_angle(i*3)
            self.car.set_cam_tilt_angle(-i*2)
            self.car.set_dir_servo_angle(i*2)
            sleep(.05)
        # Reset all servos to neutral position after thinking animation
        self.car.reset()
        if self.logger:
            self.logger.info("Keep thinking animation complete, servos reset")

    def shake_head(self):
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot shake head: No car instance available")
            return
        if self.logger:
            self.logger.info("Shaking head")
        self.car.stop()
        self.car.set_cam_pan_angle(0)
        self.car.set_cam_pan_angle(60)
        sleep(.2)
        self.car.set_cam_pan_angle(-50)
        sleep(.1)
        self.car.set_cam_pan_angle(40)
        sleep(.1)
        self.car.set_cam_pan_angle(-30)
        sleep(.1)
        self.car.set_cam_pan_angle(20)
        sleep(.1)
        self.car.set_cam_pan_angle(-10)
        sleep(.1)
        self.car.set_cam_pan_angle(10)
        sleep(.1)
        self.car.set_cam_pan_angle(-5)
        sleep(.1)
        self.car.set_cam_pan_angle(0)

    def nod(self):
        if self.car is None:
            if self.logger:
                self.logger.warning("Cannot nod: No car instance available")
            return
        if self.logger:
            self.logger.info("Nodding")
        self.car.reset()
        self.car.set_cam_tilt_angle(0)
        self.car.set_cam_tilt_angle(5)
        sleep(.1)
        self.car.set_cam_tilt_angle(-30)
        sleep(.1)
        self.car.set_cam_tilt_angle(5)
        sleep(.1)
        self.car.set_cam_tilt_angle(-30)
        sleep(.1)
        self.car.set_cam_tilt_angle(0)


    def depressed(self):
        if self.logger:
            self.logger.info("Acting depressed")
        self.car.reset()
        self.car.set_cam_tilt_angle(0)
        self.car.set_cam_tilt_angle(20)
        sleep(.22)
        self.car.set_cam_tilt_angle(-22)
        sleep(.1)
        self.car.set_cam_tilt_angle(10)
        sleep(.1)
        self.car.set_cam_tilt_angle(-22)
        sleep(.1)
        self.car.set_cam_tilt_angle(0)
        sleep(.1)
        self.car.set_cam_tilt_angle(-22)
        sleep(.1)
        self.car.set_cam_tilt_angle(-10)
        sleep(.1)
        self.car.set_cam_tilt_angle(-22)
        sleep(.1)
        self.car.set_cam_tilt_angle(-15)
        sleep(.1)
        self.car.set_cam_tilt_angle(-22)
        sleep(.1)
        self.car.set_cam_tilt_angle(-19)
        sleep(.1)
        self.car.set_cam_tilt_angle(-22)
        sleep(.1)
        sleep(1.5)
        self.car.reset()

    def twist_body(self):
        if self.logger:
            self.logger.info("Twisting body")
        self.car.reset()
        for i in range(3):
            self.car.set_motor_speed(1, 20)
            self.car.set_motor_speed(2, 20)
            self.car.set_cam_pan_angle(-20)
            self.car.set_dir_servo_angle(-10)
            sleep(.1)
            self.car.set_motor_speed(1, 0)
            self.car.set_motor_speed(2, 0)
            self.car.set_cam_pan_angle(0)
            self.car.set_dir_servo_angle(0)
            sleep(.1)
            self.car.set_motor_speed(1, -20)
            self.car.set_motor_speed(2, -20)
            self.car.set_cam_pan_angle(20)
            self.car.set_dir_servo_angle(10)
            sleep(.1)
            self.car.set_motor_speed(1, 0)
            self.car.set_motor_speed(2, 0)
            self.car.set_cam_pan_angle(0)
            self.car.set_dir_servo_angle(0)
            sleep(.1)

    def celebrate(self):
        if self.logger:
            self.logger.info("Celebrating")
        self.car.reset()
        self.car.set_cam_tilt_angle(20)

        self.car.set_dir_servo_angle(30)
        self.car.set_cam_pan_angle(60)
        sleep(.3)
        self.car.set_dir_servo_angle(10)
        self.car.set_cam_pan_angle(30)
        sleep(.1)
        self.car.set_dir_servo_angle(30)
        self.car.set_cam_pan_angle(60)
        sleep(.3)
        self.car.set_dir_servo_angle(0)
        self.car.set_cam_pan_angle(0)
        sleep(.2)

        self.car.set_dir_servo_angle(-30)
        self.car.set_cam_pan_angle(-60)
        sleep(.3)
        self.car.set_dir_servo_angle(-10)
        self.car.set_cam_pan_angle(-30)
        sleep(.1)
        self.car.set_dir_servo_angle(-30)
        self.car.set_cam_pan_angle(-60)
        sleep(.3)
        self.car.set_dir_servo_angle(0)
        self.car.set_cam_pan_angle(0)
        sleep(.2)

    def honk(self):
        if self.logger:
            self.logger.info("Honking horn")
        try:
            if hasattr(self.car, 'music'):
                self.car.music.sound_play("../sounds/car-double-horn.wav", 100)
                while self.car.music.pygame.mixer.music.get_busy():
                    time.sleep(0.1)
            else:
                if self.logger:
                    self.logger.warn("Warning: Car does not have audio capabilities")
        except Exception as e:
            if self.logger:
                self.logger.info(f"Error playing honk sound: {e}")

    def start_engine(self):
        if self.logger:
            self.logger.info("Starting engine")
        try:
            if hasattr(self.car, 'music'):
                self.car.music.sound_play("../sounds/car-start-engine.wav", 50)
                while self.car.music.pygame.mixer.music.get_busy():
                    time.sleep(0.1)
            else:
                if self.logger:
                    self.logger.warn("Warning: Car does not have audio capabilities")
        except Exception as e:
            if self.logger:
                self.logger.info(f"Error playing engine sound: {e}")

    # Actions dictionary removed - methods are now instance methods called directly
