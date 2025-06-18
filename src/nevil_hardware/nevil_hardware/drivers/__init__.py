"""
Hardware Drivers Module

This module contains concrete implementations of hardware interfaces
for different backends (physical, simulation, etc.).
"""

# Import drivers for convenience
try:
    from .simulation.simulation_movement_driver import SimulationMovementDriver
except ImportError:
    SimulationMovementDriver = None

try:
    from .physical.picarx_movement_driver import PiCarXMovementDriver
except ImportError:
    PiCarXMovementDriver = None

__all__ = []

if SimulationMovementDriver:
    __all__.append('SimulationMovementDriver')

if PiCarXMovementDriver:
    __all__.append('PiCarXMovementDriver')