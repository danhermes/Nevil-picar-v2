#!/usr/bin/env python3

import time
import functools
import threading
from typing import Callable, Any, Dict, Optional

import rclpy
from rclpy.node import Node

from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI

class NevilV1Adapter:
    """
    Compatibility adapter for Nevil v1.0 code.
    
    This class provides a drop-in replacement for the original PiCar-X
    interface used in Nevil v1.0, adapting it to use the new ROS2-based
    NevilNavigationAPI.
    """
    
    def __init__(self, node: Optional[Node] = None):
        """
        Initialize the adapter.
        
        Args:
            node: An existing ROS2 node to use. If None, a new node will be created.
        """
        # Create the API instance
        self.api = NevilNavigationAPI(node)
        
        # Set up properties to match the original PiCar-X interface
        self.speed = 50  # Default speed (0-100)
        self.SafeDistance = 50  # Safe distance in cm
        self.DangerDistance = 20  # Danger distance in cm
        
        # Convert to meters for the API
        self.api.safe_distance = self.SafeDistance / 100.0
        self.api.danger_distance = self.DangerDistance / 100.0
        
        # Mock Vilib for face detection compatibility
        self.Vilib = MockVilib()
        
        # Mock music player for sound compatibility
        self.music = MockMusicPlayer()
    
    def forward(self, speed=None):
        """Move forward at the specified speed."""
        if speed is None:
            speed = self.speed
        
        # Convert speed from 0-100 to 0-1
        normalized_speed = speed / 100.0
        
        # Create a Twist message and publish it
        self.api.move_forward_this_way(10, normalized_speed, check_obstacles=True)
    
    def backward(self, speed=None):
        """Move backward at the specified speed."""
        if speed is None:
            speed = self.speed
        
        # Convert speed from 0-100 to 0-1
        normalized_speed = speed / 100.0
        
        # Create a Twist message and publish it
        self.api.move_backward_this_way(10, normalized_speed)
    
    def stop(self):
        """Stop the robot."""
        self.api.stop()
    
    def get_distance(self):
        """Get the distance from the ultrasonic sensor in cm."""
        # Convert from meters to cm
        return self.api.get_distance() * 100.0
    
    def set_dir_servo_angle(self, angle):
        """Set the direction servo angle."""
        self.api.set_dir_servo_angle(angle)
    
    def set_cam_pan_angle(self, angle):
        """Set the camera pan angle."""
        self.api.set_cam_pan_angle(angle)
    
    def set_cam_tilt_angle(self, angle):
        """Set the camera tilt angle."""
        self.api.set_cam_tilt_angle(angle)
    
    def set_motor_speed(self, motor, speed):
        """Set the speed of a specific motor."""
        # This is a direct hardware control that we'll need to adapt
        # In the ROS2 architecture, we don't directly control motors
        # but we can simulate it for compatibility
        self.api.node.get_logger().info(f'Setting motor {motor} speed to {speed}')
        
        # If both motors are being set to the same speed, we can use forward/backward
        if motor == 1:  # Left motor
            self._left_motor_speed = speed
        elif motor == 2:  # Right motor
            self._right_motor_speed = speed
        
        # If both motors are set, we can determine the overall movement
        if hasattr(self, '_left_motor_speed') and hasattr(self, '_right_motor_speed'):
            if self._left_motor_speed > 0 and self._right_motor_speed > 0:
                # Forward
                avg_speed = (self._left_motor_speed + self._right_motor_speed) / 2
                self.forward(avg_speed)
            elif self._left_motor_speed < 0 and self._right_motor_speed < 0:
                # Backward
                avg_speed = (abs(self._left_motor_speed) + abs(self._right_motor_speed)) / 2
                self.backward(avg_speed)
            elif self._left_motor_speed == 0 and self._right_motor_speed == 0:
                # Stop
                self.stop()
    
    def reset(self):
        """Reset the robot to its default state."""
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_pan_angle(0)
        self.set_cam_tilt_angle(0)


class MockVilib:
    """
    Mock implementation of the Vilib face detection interface.
    
    This provides compatibility with the original action_helper.py
    face detection functionality.
    """
    
    def __init__(self):
        """Initialize the mock Vilib interface."""
        self.face_detect_switch_state = False
        self.detect_obj_parameter = {
            'face': 0,
            'face_x': 0,
            'face_y': 0
        }
    
    def face_detect_switch(self, state):
        """Turn face detection on or off."""
        self.face_detect_switch_state = state
        return state


class MockMusicPlayer:
    """
    Mock implementation of the music player interface.
    
    This provides compatibility with the original action_helper.py
    sound functionality.
    """
    
    def __init__(self):
        """Initialize the mock music player."""
        # Create a mock pygame mixer
        self.pygame = MockPygame()
    
    def sound_play(self, sound_file, volume=50):
        """Play a sound file."""
        print(f"Playing sound: {sound_file} at volume {volume}")
        
        # Simulate sound playing
        time.sleep(0.5)
        
        return True


class MockPygame:
    """Mock implementation of pygame for sound playback."""
    
    def __init__(self):
        """Initialize the mock pygame."""
        self.mixer = MockMixer()


class MockMixer:
    """Mock implementation of pygame mixer."""
    
    def __init__(self):
        """Initialize the mock mixer."""
        self.music = MockMusic()


class MockMusic:
    """Mock implementation of pygame music."""
    
    def __init__(self):
        """Initialize the mock music."""
        self._busy = False
    
    def get_busy(self):
        """Check if music is playing."""
        # Simulate music finishing after a short time
        if self._busy:
            self._busy = False
            return True
        return False


# Create compatibility functions that match the original action_helper.py

def with_obstacle_check(func):
    """
    Decorator to add obstacle checking to movement functions.
    
    This is a compatibility wrapper for the original decorator in action_helper.py.
    """
    @functools.wraps(func)
    def wrapper(car, *args, **kwargs):
        def check_distance():
            distance = car.get_distance()
            if distance >= car.SafeDistance:
                return "safe"
            elif distance >= car.DangerDistance:
                car.set_dir_servo_angle(30)
                return "caution"
            else:
                car.set_dir_servo_angle(-30)
                move_backward_this_way(car, 10, car.speed)
                time.sleep(0.5)
                return "danger"
        
        return func(car, *args, check_distance=check_distance, **kwargs)
    
    return wrapper


@with_obstacle_check
def move_forward_this_way(car, distance_cm, speed=None, check_distance=None):
    """Move forward a specific distance at given speed."""
    if speed is None:
        speed = car.speed
    
    # Convert speed from 0-100 to 0-1
    normalized_speed = speed / 100.0
    
    # Use the API to move forward
    return car.api.move_forward_this_way(distance_cm, normalized_speed, check_obstacles=True)


def move_backward_this_way(car, distance_cm, speed=None):
    """Move backward a specific distance at given speed."""
    if speed is None:
        speed = car.speed
    
    # Convert speed from 0-100 to 0-1
    normalized_speed = speed / 100.0
    
    # Use the API to move backward
    return car.api.move_backward_this_way(distance_cm, normalized_speed)


def turn_left(car):
    """Turn the robot left."""
    return car.api.turn_left()


def turn_right(car):
    """Turn the robot right."""
    return car.api.turn_right()


def stop(car):
    """Stop the robot."""
    return car.api.stop()


def turn_left_in_place(car):
    """Turn the robot left in place."""
    return car.api.set_dir_servo_angle(-30)


def turn_right_in_place(car):
    """Turn the robot right in place."""
    return car.api.set_dir_servo_angle(30)


# @with_obstacle_check
# def come_here(car, check_distance=None):
#     """Make the robot come to a detected face."""
#     # This is a complex behavior that uses face detection
#     # For now, we'll just call the API's perform_behavior method
#     return car.api.perform_behavior('come_here')


def clamp_number(num, a, b):
    """Clamp a number between a and b."""
    return max(min(num, max(a, b)), min(a, b))


def wave_hands(car):
    """Make the robot wave its hands."""
    return car.api.wave_hands()


def resist(car):
    """Make the robot resist."""
    return car.api.resist()


def act_cute(car):
    """Make the robot act cute."""
    return car.api.act_cute()


def rub_hands(car):
    """Make the robot rub its hands."""
    return car.api.rub_hands()


def think(car):
    """Make the robot think."""
    return car.api.think()


def keep_think(car):
    """Make the robot keep thinking."""
    return car.api.keep_think()


def shake_head(car):
    """Make the robot shake its head."""
    return car.api.shake_head()


def nod(car):
    """Make the robot nod."""
    return car.api.nod()


def depressed(car):
    """Make the robot look depressed."""
    return car.api.depressed()


def twist_body(car):
    """Make the robot twist its body."""
    return car.api.twist_body()


def celebrate(car):
    """Make the robot celebrate."""
    return car.api.celebrate()


def honk(car):
    """Make the robot honk."""
    return car.api.honk()


def start_engine(car):
    """Make the robot start its engine sound."""
    return car.api.start_engine()


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
    "keep think": keep_think,
    "honk": honk,
    "start engine": start_engine
}