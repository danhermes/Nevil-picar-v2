"""
Nevil-picar v2.0 Text and Voice Interfaces Package.

This package provides natural language interaction capabilities for
controlling and communicating with the Nevil-picar v2.0 robot.
"""

import os
import logging
from pathlib import Path
try:
    from dotenv import load_dotenv
    DOTENV_AVAILABLE = True
except ImportError:
    DOTENV_AVAILABLE = False

__version__ = '0.1.0'

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('nevil_interfaces_ai')

def load_env_file(env_file_path=None):
    """
    Load environment variables from a .env file.
    
    Args:
        env_file_path: Path to the .env file (default: project root .env)
        
    Returns:
        dict: Dictionary of environment variables loaded from the file
    """
    # If python-dotenv is available, use it
    if DOTENV_AVAILABLE:
        # If no path is provided, look for .env in the project root
        if env_file_path is None:
            # Try to find the project root by looking for .env file
            current_dir = Path.cwd()
            while current_dir != current_dir.parent:
                env_path = current_dir / '.env'
                if env_path.exists():
                    env_file_path = env_path
                    break
                current_dir = current_dir.parent
        
        # If still None, use the current directory
        if env_file_path is None:
            env_file_path = Path.cwd() / '.env'
        
        # Convert to Path object if it's a string
        if isinstance(env_file_path, str):
            env_file_path = Path(env_file_path)
        
        # Load environment variables from the .env file
        if env_file_path.exists():
            logger.info(f'Loading environment variables from {env_file_path}')
            load_dotenv(dotenv_path=env_file_path)
        else:
            logger.warning(f'.env file not found at {env_file_path}')
    else:
        # Fallback to manual loading if python-dotenv is not available
        logger.warning('python-dotenv not available, falling back to manual loading')
        # If no path is provided, look for .env in the project root
        if env_file_path is None:
            # Try to find the project root by looking for .env file
            current_dir = Path.cwd()
            while current_dir != current_dir.parent:
                env_path = current_dir / '.env'
                if env_path.exists():
                    env_file_path = env_path
                    break
                current_dir = current_dir.parent
        
        # If still None, use the current directory
        if env_file_path is None:
            env_file_path = Path.cwd() / '.env'
        
        # Convert to Path object if it's a string
        if isinstance(env_file_path, str):
            env_file_path = Path(env_file_path)
        
        # Load environment variables from the .env file
        if env_file_path.exists():
            logger.info(f'Loading environment variables from {env_file_path}')
            with open(env_file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    key, value = line.split('=', 1)
                    os.environ[key.strip()] = value.strip()
        else:
            logger.warning(f'.env file not found at {env_file_path}')
    
    # Return a dictionary of all environment variables
    return {key: value for key, value in os.environ.items()}

def get_env_var(name, default=None, env_vars=None):
    """
    Get an environment variable, with fallback to a default value.
    
    Args:
        name: Name of the environment variable
        default: Default value if the environment variable is not set
        env_vars: Dictionary of environment variables (optional)
        
    Returns:
        The value of the environment variable, or the default value
    """
    # First check the provided env_vars dictionary
    if env_vars and name in env_vars:
        return env_vars[name]
    
    # Then check the environment
    return os.environ.get(name, default)

# Load environment variables from .env file
env_vars = load_env_file()

# Import hardware interfaces
from nevil_interfaces_ai.audio_hardware_interface import AudioHardwareInterface