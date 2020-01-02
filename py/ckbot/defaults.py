"""
FILE: defaults.py

Contains configuration defaults for logical.py

This file exists so you won't need to edit core files to change
default behavior of applications using logical.py and related classes
"""
from . import dynamixel

# Default cluster hardware architecture
DEFAULT_ARCH = dynamixel
# Default interface to use -- punt to architecture
DEFAULT_PORT = None
