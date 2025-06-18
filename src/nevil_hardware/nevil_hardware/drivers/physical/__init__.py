"""
Physical Hardware Drivers Module

This module contains implementations for actual physical hardware.
"""

# Import will be available when physical driver is implemented
try:
    from .picarx_movement_driver import PiCarXMovementDriver
    __all__ = ['PiCarXMovementDriver']
except ImportError:
    # Physical driver not yet implemented or dependencies not available
    __all__ = []