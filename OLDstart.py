#!/usr/bin/env python3

import os
import sys
import subprocess
import platform
import shutil
from pathlib import Path

class NevilSetup:
    def __init__(self):
        self.workspace_root = Path(__file__).resolve().parent
        self.src_dir = self.workspace_root / "src"
        self.setup_file = self.workspace_root / "install" / "setup.bash"
        
        # Determine ROS2 distribution based on system
        self.ros_distro = self.determine_ros_distro()
        
    def determine_ros_distro(self):
        """Determine the appropriate ROS2 distribution based on the system"""
        # Check OS and version
        os_info = self.get_os_info()
        arch = platform.machine()
        
        print(f"Detected OS: {os_info['id']} {os_info['version_id']} ({arch})")
        
        # Check if we're on a Raspberry Pi
        is_raspberry_pi = self.is_raspberry_pi()
        if is_raspberry_pi:
            print("Detected Raspberry Pi hardware")
            
            # For Raspberry Pi with Debian Bookworm, Humble is more stable
            if os_info['id'] == 'debian' and os_info['version_id'] == '12':
                return 'humble'
        
        # Default to Humble as it has wider compatibility
        # Iron is newer but might have limited platform support
        if os_info['id'] == 'debian':
            if os_info['version_id'] == '12':  # Bookworm
                # For Debian Bookworm, we'll try Humble as it's more stable
                return 'humble'
            else:
                return 'humble'
        elif os_info['id'] == 'ubuntu':
            if os_info['version_id'] in ['22.04', '24.04']:
                return 'iron'
            else:
                return 'humble'
        else:
            # Default to Humble for other systems
            return 'humble'
    
    def is_raspberry_pi(self):
        """Check if we're running on a Raspberry Pi"""
        try:
            # Check for Raspberry Pi model in /proc/cpuinfo
            if os.path.exists('/proc/cpuinfo'):
                with open('/proc/cpuinfo', 'r') as f:
                    cpuinfo = f.read()
                    return 'Raspberry Pi' in cpuinfo or 'BCM' in cpuinfo or 'Broadcom' in cpuinfo
            return False
        except:
            return False
    
    def get_os_info(self):
        """Get OS information"""
        os_info = {'id': '', 'version_id': ''}
        
        try:
            # Try to read /etc/os-release
            if os.path.exists('/etc/os-release'):
                with open('/etc/os-release', 'r') as f:
                    for line in f:
                        if line.startswith('ID='):
                            os_info['id'] = line.split('=')[1].strip().strip('"')
                        elif line.startswith('VERSION_ID='):
                            os_info['version_id'] = line.split('=')[1].strip().strip('"')
            else:
                # Fallback to platform module
                system = platform.system().lower()
                if system == 'linux':
                    dist_info = platform.freedesktop_os_release()
                    os_info['id'] = dist_info.get('ID', '')
                    os_info['version_id'] = dist_info.get('VERSION_ID', '')
                else:
                    os_info['id'] = system
                    os_info['version_id'] = platform.version()
        except Exception as e:
            print(f"Warning: Could not determine OS info: {e}")
            # Use platform module as fallback
            os_info['id'] = platform.system().lower()
            os_info['version_id'] = platform.version()
            
        return os_info
    
    def run_command(self, cmd, shell=False, check=True):
        """Run a command and return its output"""
        try:
            if shell:
                result = subprocess.run(cmd, shell=True, check=check, text=True, capture_output=True)
            else:
                result = subprocess.run(cmd if isinstance(cmd, list) else cmd.split(), 
                                       check=check, text=True, capture_output=True)
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"Error executing command: {cmd}")
            print(f"Error output: {e.stderr}")
            if check:
                raise
            return e.stderr
    
    def check_python(self):
        """Check for Python and install if needed"""
        print("Checking Python installation...")
        try:
            python_version = self.run_command("python3 --version")
            print(f"Found {python_version.strip()}")
        except:
            print("Installing Python3...")
            self.run_command("sudo apt update && sudo apt install -y python3 python3-pip", shell=True)
    
    def setup_ros_repository(self):
        """Set up the ROS2 repository"""
        print(f"Setting up ROS2 {self.ros_distro} repository...")
        
        # Check if we're on aarch64 (ARM64) architecture
        arch = platform.machine()
        if arch == 'aarch64':
            print("Detected ARM64 architecture (aarch64)")
            print("Standard ROS2 repositories don't support aarch64 on Debian.")
            print("Will use alternative installation method.")
            
            # For ARM64, we'll use a different approach
            return self.setup_ros_for_arm64()
        
        # Standard x86_64 approach
        # Clean up any existing ROS2 repository files
        print("Cleaning up old ROS2 repositories...")
        self.run_command("sudo rm -f /etc/apt/sources.list.d/ros2*.list", shell=True)
        
        # Install prerequisites
        print("Installing prerequisites...")
        self.run_command("sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release", shell=True)
        
        # Add ROS2 apt repository key
        print("Adding ROS2 apt repository key...")
        self.run_command("sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg")
        
        # Determine Ubuntu codename based on ROS2 distribution
        ubuntu_codename = "jammy"  # Default for Humble (Ubuntu 22.04)
        if self.ros_distro == 'iron':
            ubuntu_codename = "noble"  # Ubuntu 24.04
        
        # Add ROS2 repository
        repo_cmd = f'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu {ubuntu_codename} main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'
        self.run_command(repo_cmd, shell=True)
        
        # Update package lists
        print("Updating package lists...")
        self.run_command("sudo apt update", shell=True)
        
        return True
    
    def setup_ros_for_arm64(self):
        """Set up ROS2 for ARM64 architecture"""
        print("Setting up ROS2 for ARM64 architecture...")
        
        # Install basic dependencies from apt
        print("Installing basic build dependencies...")
        self.run_command("sudo apt update && sudo apt install -y build-essential cmake git python3-pip wget", shell=True)
        
        # Install Python dependencies using pip
        print("Installing Python dependencies...")
        self.run_command("pip3 install -U colcon-common-extensions vcstool rosdep", shell=True, check=False)
        
        # Create a script to build ROS2 from source
        build_script = self.workspace_root / "build_ros2_arm64.sh"
        with open(build_script, 'w') as f:
            f.write("""#!/bin/bash
set -e

# This script builds ROS2 from source for ARM64 architecture
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS2_WS="$SCRIPT_DIR/ros2_ws"

# Create workspace
mkdir -p $ROS2_WS/src
cd $ROS2_WS

# Get ROS2 sources
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos

# Install dependencies
sudo apt update
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build ROS2
colcon build --symlink-install

# Create setup script
echo '#!/bin/bash
# Source this file to set up ROS2 environment
source "'$ROS2_WS'"/install/setup.bash
' > $SCRIPT_DIR/setup_ros2.sh
chmod +x $SCRIPT_DIR/setup_ros2.sh

echo "ROS2 built successfully. Source $SCRIPT_DIR/setup_ros2.sh to use it."
""")
        os.chmod(build_script, 0o755)
        
        # Check available disk space
        try:
            import shutil
            total, used, free = shutil.disk_usage(self.workspace_root)
            free_gb = free // (1024 ** 3)
            
            if free_gb < 20:
                print(f"\nWARNING: You have only {free_gb}GB of free disk space.")
                print("Building ROS2 from source requires at least 20GB of free space.")
                print("Consider:")
                print("  - Using a larger SD card (64GB+)")
                print("  - Using external storage (USB drive)")
                print("  - Cleaning up unnecessary packages: sudo apt clean && sudo apt autoremove")
        except Exception as e:
            print(f"Warning: Could not check disk space: {e}")
        
        print(f"""
To build ROS2 for ARM64, run the following script:
{build_script}

This will build ROS2 Humble from source, which may take several hours.
The build process requires at least 20GB of free disk space.
After building, source the setup_ros2.sh script to use ROS2.
""")
        
        return True
    
    def install_ros(self):
        """Install ROS2"""
        print(f"Installing ROS2 {self.ros_distro}...")
        
        # Check if we're on ARM64 architecture
        if platform.machine() == 'aarch64':
            print("ARM64 architecture detected. Binary packages are not available.")
            print("Please use the build script to build ROS2 from source.")
            print(f"Run: {self.workspace_root}/build_ros2_arm64.sh")
            
            # Install development tools
            self.install_dev_tools()
            return
        
        # For x86_64 architecture, try binary packages
        # First try: Install the full desktop version (if available)
        try:
            print(f"Attempting to install ros-{self.ros_distro}-desktop...")
            result = self.run_command(f"sudo apt install -y ros-{self.ros_distro}-desktop", shell=True, check=False)
            if "Unable to locate package" not in result:
                print("ROS2 desktop installation successful!")
                self.install_dev_tools()
                return
            else:
                print("Desktop installation not available.")
        except Exception as e:
            print(f"Desktop installation failed: {e}")
        
        # Second try: Install the base version
        try:
            print(f"Attempting to install ros-{self.ros_distro}-ros-base...")
            result = self.run_command(f"sudo apt install -y ros-{self.ros_distro}-ros-base", shell=True, check=False)
            if "Unable to locate package" not in result:
                print("ROS2 base installation successful!")
                self.install_dev_tools()
                return
            else:
                print("Base installation not available.")
        except Exception as e:
            print(f"Base installation failed: {e}")
        
        # Third try: Install core packages only
        try:
            print(f"Attempting to install ros-{self.ros_distro}-ros-core...")
            result = self.run_command(f"sudo apt install -y ros-{self.ros_distro}-ros-core", shell=True, check=False)
            if "Unable to locate package" not in result:
                print("ROS2 core installation successful!")
                self.install_dev_tools()
                return
            else:
                print("Core installation not available.")
        except Exception as e:
            print(f"Core installation failed: {e}")
        
        # If all binary installations fail, offer to build from source
        print("\nBinary packages for ROS2 are not available for your system.")
        print("Would you like to build ROS2 from source? (y/n)")
        choice = input().strip().lower()
        
        if choice == 'y':
            self.build_ros_from_source()
        else:
            print("ROS2 installation skipped. You may need to install ROS2 manually.")
            print("Visit: https://docs.ros.org/en/humble/Installation.html")
            
            # Continue with the rest of the setup
            print("Continuing with workspace setup...")
    
    def setup_venv(self):
        """Set up a Python virtual environment for development tools"""
        print("Setting up Python virtual environment...")
        venv_dir = self.workspace_root / "nevil_venv"
        
        # Create venv if it doesn't exist
        if not venv_dir.exists():
            try:
                # Make sure python3-venv is installed
                self.run_command("sudo apt install -y python3-venv python3-full", shell=True)
                # Create the virtual environment
                self.run_command(f"python3 -m venv {venv_dir}", shell=True)
                print(f"Created virtual environment at {venv_dir}")
            except Exception as e:
                print(f"Failed to create virtual environment: {e}")
                return None
        
        # Check if the venv was created successfully
        python_path = venv_dir / "bin" / "python"
        pip_path = venv_dir / "bin" / "pip"
        
        # Verify the paths exist
        if not python_path.exists():
            print(f"Error: Python executable not found at {python_path}")
            return None
            
        if not pip_path.exists():
            print(f"Error: Pip executable not found at {pip_path}")
            # Try to create pip if it doesn't exist
            try:
                self.run_command(f"{python_path} -m ensurepip", shell=True)
                if not pip_path.exists():
                    return None
            except Exception as e:
                print(f"Failed to create pip: {e}")
                return None
        
        # Upgrade pip in the venv
        try:
            self.run_command(f"{python_path} -m pip install --upgrade pip", shell=True)
        except Exception as e:
            print(f"Warning: Could not upgrade pip: {e}")
        
        # Test the venv by installing a simple package
        try:
            self.run_command(f"{pip_path} install --upgrade setuptools wheel", shell=True)
            print("Virtual environment is working correctly.")
        except Exception as e:
            print(f"Warning: Virtual environment may not be working correctly: {e}")
            # Continue anyway
        
        return {
            'python': str(python_path),
            'pip': str(pip_path),
            'venv_dir': str(venv_dir)
        }
    
    def install_dev_tools(self):
        """Install development tools"""
        print("Installing colcon and development tools...")
        
        # Try apt first
        try:
            self.run_command("sudo apt install -y python3-colcon-common-extensions python3-rosdep", shell=True, check=False)
        except Exception:
            print("Could not install development tools via apt.")
        
        # Check if colcon and rosdep are installed
        colcon_installed = self.check_command_exists("colcon")
        rosdep_installed = self.check_command_exists("rosdep")
        
        # If either tool is missing, try to install in a venv
        if not (colcon_installed and rosdep_installed):
            print("Some development tools are missing. Setting up virtual environment...")
            venv = self.setup_venv()
            
            if venv:
                # Install tools in the venv
                if not colcon_installed:
                    print("Installing colcon in virtual environment...")
                    try:
                        self.run_command(f"{venv['pip']} install -U colcon-common-extensions", shell=True)
                        print("Colcon installed successfully in virtual environment.")
                        
                        # Create a wrapper script for colcon
                        colcon_wrapper = self.workspace_root / "colcon_wrapper.sh"
                        with open(colcon_wrapper, 'w') as f:
                            f.write(f"""#!/bin/bash
# Wrapper script for colcon in virtual environment
{venv['venv_dir']}/bin/colcon "$@"
""")
                        os.chmod(colcon_wrapper, 0o755)
                        print(f"Created colcon wrapper script at {colcon_wrapper}")
                        
                        # Add to PATH in bashrc
                        bashrc_path = Path.home() / ".bashrc"
                        path_line = f'export PATH="{self.workspace_root}:$PATH"'
                        
                        if bashrc_path.exists():
                            with open(bashrc_path, 'r') as f:
                                if path_line not in f.read():
                                    with open(bashrc_path, 'a') as f:
                                        f.write(f"\n# Nevil development tools\n{path_line}\n")
                                        print("Added development tools to PATH in ~/.bashrc")
                    except Exception as e:
                        print(f"Failed to install colcon in virtual environment: {e}")
                
                if not rosdep_installed:
                    print("Installing rosdep in virtual environment...")
                    try:
                        self.run_command(f"{venv['pip']} install -U rosdep", shell=True)
                        print("Rosdep installed successfully in virtual environment.")
                        
                        # Create a wrapper script for rosdep
                        rosdep_wrapper = self.workspace_root / "rosdep_wrapper.sh"
                        with open(rosdep_wrapper, 'w') as f:
                            f.write(f"""#!/bin/bash
# Wrapper script for rosdep in virtual environment
{venv['venv_dir']}/bin/rosdep "$@"
""")
                        os.chmod(rosdep_wrapper, 0o755)
                        print(f"Created rosdep wrapper script at {rosdep_wrapper}")
                    except Exception as e:
                        print(f"Failed to install rosdep in virtual environment: {e}")
            else:
                print("Could not set up virtual environment. You may need to install development tools manually.")
    
    def check_command_exists(self, command):
        """Check if a command exists in the system"""
        try:
            result = self.run_command(f"which {command}", shell=True, check=False)
            return bool(result.strip())
        except Exception:
            return False
    
    def build_ros_from_source(self):
        """Build ROS2 from source"""
        print("Building ROS2 from source. This may take a while...")
        
        # Create a directory for the ROS2 source
        ros_src_dir = self.workspace_root / "ros2_src"
        if not ros_src_dir.exists():
            ros_src_dir.mkdir(parents=True)
        
        # Install dependencies for building ROS2
        print("Installing build dependencies...")
        self.run_command("sudo apt update && sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool wget", shell=True)
        self.run_command("python3 -m pip install -U rosinstall_generator", shell=True)
        
        # Download ROS2 source
        os.chdir(ros_src_dir)
        print(f"Downloading ROS2 {self.ros_distro} source...")
        self.run_command(f"wget https://raw.githubusercontent.com/ros2/ros2/{self.ros_distro}/ros2.repos", shell=True)
        self.run_command("vcs import src < ros2.repos", shell=True)
        
        # Install dependencies
        print("Installing dependencies for building ROS2...")
        self.run_command("sudo rosdep init || true", shell=True)
        self.run_command("rosdep update", shell=True)
        self.run_command("rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys \"fastcdr rti-connext-dds-6.0.1 urdfdom_headers\"", shell=True)
        
        # Build ROS2
        print("Building ROS2 (this will take a while)...")
        self.run_command("colcon build --symlink-install", shell=True)
        
        # Create setup script
        setup_script = ros_src_dir / "setup_ros2.sh"
        with open(setup_script, 'w') as f:
            f.write(f"""#!/bin/bash
# Source this file to set up ROS2 environment
source {ros_src_dir}/install/setup.bash
""")
        os.chmod(setup_script, 0o755)
        
        print(f"ROS2 built successfully. Source {setup_script} to use it.")
        
        # Add to bashrc
        bashrc_path = Path.home() / ".bashrc"
        source_line = f"source {setup_script}"
        
        if bashrc_path.exists():
            with open(bashrc_path, 'r') as f:
                if source_line not in f.read():
                    with open(bashrc_path, 'a') as f:
                        f.write(f"\n# ROS2 built from source\n{source_line}\n")
                        print("Added ROS2 source to ~/.bashrc")
    
    def setup_workspace(self):
        """Set up the workspace structure"""
        print("Setting up workspace structure...")
        
        # Create src directory if it doesn't exist
        if not self.src_dir.exists():
            self.src_dir.mkdir(parents=True)
        
        # Check if rosdep is available
        if self.check_command_exists("rosdep"):
            # Initialize rosdep if needed
            print("Initializing rosdep...")
            try:
                self.run_command("sudo rosdep init", shell=True, check=False)
            except:
                # rosdep init might fail if already initialized, which is fine
                pass
            
            try:
                self.run_command("rosdep update", shell=True)
            except Exception as e:
                print(f"Warning: Could not update rosdep: {e}")
            
            # Install dependencies
            print("Installing dependencies...")
            try:
                self.run_command("rosdep install --from-paths src -y --ignore-src", shell=True)
            except Exception as e:
                print(f"Warning: Some dependencies could not be installed: {e}")
                print("Continuing anyway...")
        else:
            print("Warning: rosdep is not available. Skipping dependency installation.")
            print("You may need to install dependencies manually.")
    
    def build_workspace(self):
        """Build the workspace with colcon"""
        print("Building workspace with colcon...")
        os.chdir(self.workspace_root)
        
        # Check if colcon is available
        if self.check_command_exists("colcon"):
            try:
                self.run_command("colcon build", shell=True)
            except subprocess.CalledProcessError:
                print("Build failed. Please check the errors above.")
                print("Continuing with setup...")
        else:
            print("Warning: colcon is not available. Skipping workspace build.")
            print("You may need to install colcon and build the workspace manually.")
    
    def source_workspace(self):
        """Source the workspace and add to bashrc"""
        if self.setup_file.exists():
            print("Sourcing the workspace...")
            
            # We can't directly source in Python, so we'll add to bashrc
            bashrc_path = Path.home() / ".bashrc"
            source_line = f"source {self.setup_file}"
            
            # Check if already in bashrc
            if bashrc_path.exists():
                with open(bashrc_path, 'r') as f:
                    if source_line not in f.read():
                        with open(bashrc_path, 'a') as f:
                            f.write(f"\n# Nevil workspace\n{source_line}\n")
                            print("Added workspace to ~/.bashrc")
            else:
                with open(bashrc_path, 'w') as f:
                    f.write(f"# Nevil workspace\n{source_line}\n")
                    print("Created ~/.bashrc with workspace source")
        else:
            print(f"Warning: Setup file not found at {self.setup_file}")
    
    def source_ros2_setup(self):
        """Create a script to source ROS2 setup files"""
        print("Creating ROS2 setup script...")
        
        # Check for ROS2 setup files in standard locations
        setup_paths = [
            "/opt/ros/humble/setup.bash",
            "/opt/ros/iron/setup.bash"
        ]
        
        # Find the first existing setup file
        ros_setup_path = None
        for path in setup_paths:
            if os.path.exists(path):
                ros_setup_path = path
                break
        
        if ros_setup_path:
            # Create a setup script
            setup_script = self.workspace_root / "setup_ros2.sh"
            with open(setup_script, 'w') as f:
                f.write(f"""#!/bin/bash
# Source this file to set up ROS2 environment
source {ros_setup_path}
""")
            os.chmod(setup_script, 0o755)
            
            print(f"Created ROS2 setup script at {setup_script}")
            
            # Add to bashrc
            bashrc_path = Path.home() / ".bashrc"
            source_line = f"source {setup_script}"
            
            if bashrc_path.exists():
                with open(bashrc_path, 'r') as f:
                    if source_line not in f.read():
                        with open(bashrc_path, 'a') as f:
                            f.write(f"\n# ROS2 environment\n{source_line}\n")
                            print("Added ROS2 setup to ~/.bashrc")
            
            return True
        else:
            print("Could not find ROS2 setup files.")
            return False
    
    def check_ros2_installation(self):
        """Check if ROS2 is installed and available"""
        # First check if ros2 command exists
        if self.check_command_exists("ros2"):
            print("ros2 command found in PATH.")
            return True
        
        # If command doesn't exist, check if ROS2 packages are installed
        try:
            # Use apt list to check for installed ROS2 packages (more reliable)
            result = self.run_command("apt list --installed 2>/dev/null | grep -i 'ros-humble'", shell=True, check=False)
            if result and "ros-humble" in result:
                print("ROS2 Humble packages are installed, but ros2 command is not in PATH.")
                print("This is likely because the ROS2 setup file has not been sourced.")
                return True
            
            # Check for Iron packages as well
            result = self.run_command("apt list --installed 2>/dev/null | grep -i 'ros-iron'", shell=True, check=False)
            if result and "ros-iron" in result:
                print("ROS2 Iron packages are installed, but ros2 command is not in PATH.")
                print("This is likely because the ROS2 setup file has not been sourced.")
                return True
            
            # Check for specific ROS2 packages using dpkg as a fallback
            for pkg in ["ros-humble-desktop", "ros-humble-ros-base", "ros-humble-ros-core", "ros-iron-desktop", "ros-iron-ros-base", "ros-iron-ros-core"]:
                result = self.run_command(f"dpkg -l {pkg} 2>/dev/null | grep -i '{pkg}'", shell=True, check=False)
                if result and pkg in result:
                    print(f"{pkg} is installed, but ros2 command is not in PATH.")
                    return True
            
            # Check if ROS2 setup files exist
            for path in ["/opt/ros/humble/setup.bash", "/opt/ros/iron/setup.bash"]:
                if os.path.exists(path):
                    print(f"ROS2 setup file found at {path}, but ros2 command is not in PATH.")
                    print(f"You need to source this file: source {path}")
                    return True
            
            # Check for any ROS2 directories
            if os.path.exists("/opt/ros/humble") or os.path.exists("/opt/ros/iron"):
                print("ROS2 installation directory found, but setup may be incomplete.")
                return True
            
            return False
        except Exception as e:
            print(f"Error checking ROS2 installation: {e}")
            return False
    
    def run_setup(self):
        """Run the complete setup process"""
        print("Starting comprehensive setup for Nevil...")
        
        self.check_python()
        
        # Check if ROS2 is already installed
        ros2_installed = self.check_ros2_installation()
        if ros2_installed:
            print("ROS2 is already installed!")
            
            # If ROS2 is installed but not in PATH, create a setup script and source it
            if not self.check_command_exists("ros2"):
                print("Setting up ROS2 environment...")
                self.source_ros2_setup()
                
                # Try to source the ROS2 setup file directly for this session
                ros_setup_paths = [
                    "/opt/ros/humble/setup.bash",
                    "/opt/ros/iron/setup.bash"
                ]
                
                for path in ros_setup_paths:
                    if os.path.exists(path):
                        print(f"Sourcing {path} for this session...")
                        # We can't directly source in Python, but we can create a temporary script
                        temp_script = self.workspace_root / "temp_source.sh"
                        with open(temp_script, 'w') as f:
                            f.write(f"""#!/bin/bash
source {path}
echo "ROS2_DISTRO=$ROS_DISTRO"
echo "PATH=$PATH"
""")
                        os.chmod(temp_script, 0o755)
                        
                        # Run the script and capture output
                        try:
                            result = self.run_command(f"bash {temp_script}", shell=True)
                            print(f"ROS2 environment sourced: {result}")
                            # Clean up
                            os.remove(temp_script)
                        except Exception as e:
                            print(f"Warning: Could not source ROS2 setup file: {e}")
                        
                        break
        else:
            print("ROS2 is not installed. Setting up repositories...")
            self.setup_ros_repository()
            self.install_ros()
        
        # Continue with workspace setup even if ROS2 installation was skipped
        self.setup_workspace()
        
        # Only build if ROS2 is available
        if self.check_ros2_installation():
            self.build_workspace()
            self.source_workspace()
            
            print(f"""
Setup completed successfully!

Your workspace is now ready to use. A new terminal will automatically source
the workspace, or you can run:
source ~/.bashrc

To rebuild the workspace in the future, run:
cd {self.workspace_root} && colcon build

ROS2 distribution: {self.ros_distro}
            """)
        else:
            # Create a helper script to run after sourcing ROS2
            helper_script = self.workspace_root / "complete_setup.sh"
            with open(helper_script, 'w') as f:
                f.write(f"""#!/bin/bash
# This script will complete the Nevil setup after sourcing ROS2

echo "Completing Nevil setup with ROS2 environment..."

# Source ROS2 setup
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
else
    echo "Error: ROS2 setup file not found!"
    exit 1
fi

# Run the setup script again
cd {self.workspace_root}
python3 {self.workspace_root}/start.py

echo "Setup completed!"
""")
            os.chmod(helper_script, 0o755)
            
            print(f"""
Setup partially completed.

ROS2 was installed but needs to be sourced. To complete the setup:

1. Open a new terminal
2. Run the following command:
   {helper_script}

This will source the ROS2 environment and complete the setup.

Your workspace structure has been created at:
{self.workspace_root}
            """)

if __name__ == "__main__":
    setup = NevilSetup()
    setup.run_setup()