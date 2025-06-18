"""
Nevil Hardware Package v2.0

This package provides hardware abstraction layer for the Nevil-picar robot,
enabling clean separation between physical hardware and simulation modes.

Key Components:
- MovementInterface: Abstract base class for movement control
- HardwareManager: Factory for creating hardware interfaces
- Physical Drivers: Real hardware implementations
- Simulation Drivers: Virtual hardware implementations

Usage:
    from nevil_hardware.hardware_manager import HardwareManager
    
    manager = HardwareManager()
    movement = manager.get_movement_interface()
    movement.forward(0.5)
"""

__version__ = "2.0.0"
__author__ = "Nevil Team"
__email__ = "nevil@example.com"

# Import main classes for convenience
from .hardware_manager import HardwareManager
from .interfaces.movement_interface import MovementInterface, MovementStatus

__all__ = [
    'HardwareManager',
    'MovementInterface', 
    'MovementStatus'
]