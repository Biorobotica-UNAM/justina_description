#!/bin/bash

# virtual_envs/install.sh - Install simulation dependencies

set -e  # Exit on error

echo "======================================"
echo "Installing Justina Simulation Dependencies"
echo "======================================"

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install ROS 2 Jazzy packages
echo "Installing ROS 2 Jazzy packages..."
sudo apt install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro

# Optional but useful packages
echo "Installing optional useful packages..."
sudo apt install -y \
    ros-jazzy-launch-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-topic-tools \
    ros-jazzy-webots-ros2

# Install Gazebo Sim if not already installed
echo "Checking for Gazebo Sim..."
if ! command -v gz &> /dev/null; then
    echo "Installing Gazebo Sim..."
    sudo apt install -y gz-sim
else
    echo "✓ Gazebo Sim already installed"
fi

# ==========================================
# Install Webots R2025a
# ==========================================
echo "Checking for Webots..."
if ! command -v webots &> /dev/null; then
    echo "Adding Webots APT repository..."
    sudo apt install -y wget gnupg
    wget -qO- https://www.cyberbotics.com/apt/key.asc | sudo tee /etc/apt/trusted.gpg.d/webots.asc > /dev/null
    echo "deb https://www.cyberbotics.com/apt/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/webots.list

    echo "Updating package lists with Webots repository..."
    sudo apt update

    echo "Installing Webots R2025a..."
    sudo apt install -y webots
else
    echo "✓ Webots already installed"
fi

# Determine the actual Webots installation directory
# For apt packages it's usually /usr/share/webots
# But we'll try to locate it by looking for the lib/controller/cmake subdirectory
WEBOTS_INSTALL_DIR=""
if dpkg -l webots &> /dev/null; then
    # If installed via apt, get the directory from package info
    WEBOTS_INSTALL_DIR=$(dpkg -L webots | grep -m1 "/usr/share/webots$" || true)
fi
if [ -z "$WEBOTS_INSTALL_DIR" ] && [ -d "/usr/share/webots/lib/controller/cmake" ]; then
    WEBOTS_INSTALL_DIR="/usr/share/webots"
fi
if [ -z "$WEBOTS_INSTALL_DIR" ] && [ -d "/usr/local/webots/lib/controller/cmake" ]; then
    WEBOTS_INSTALL_DIR="/usr/local/webots"
fi
if [ -z "$WEBOTS_INSTALL_DIR" ]; then
    echo "Warning: Could not auto-detect Webots installation directory."
    echo "Using default: /usr/local/webots (you may need to adjust manually)."
    WEBOTS_INSTALL_DIR="/usr/local/webots"
fi

echo "Webots installation directory detected: $WEBOTS_INSTALL_DIR"

# Add environment variables to ~/.bashrc if not already present
ENV_VARS_EXPORT="export WEBOTS_HOME=$WEBOTS_INSTALL_DIR
export Webots_DIR=\$WEBOTS_HOME/lib/controller/cmake"

if ! grep -q "WEBOTS_HOME" ~/.bashrc; then
    echo "Adding Webots environment variables to ~/.bashrc..."
    echo -e "\n# Webots environment variables" >> ~/.bashrc
    echo "$ENV_VARS_EXPORT" >> ~/.bashrc
    echo "Done. Please run 'source ~/.bashrc' or open a new terminal to apply changes."
else
    echo "Webots environment variables already present in ~/.bashrc (skipping)."
fi

# ==========================================

# Install Gazebo message tools (for debugging)
echo "Installing Gazebo message tools..."
#sudo apt install -y gz-msgs9 gz-tools

# Install Python packages for teleoperation
echo "Installing Python packages for teleoperation..."
sudo apt install -y python3-pygame
sudo apt install ros-jazzy-joy
sudo apt install ros-jazzy-teleop-twist-joy

# Clean up
echo "Cleaning up..."
sudo apt autoremove -y

# Verify installations
echo ""
echo "======================================"
echo "Verification"
echo "======================================"

# Check key commands
echo "Checking key commands:"
commands=("gz" "ros2" "xacro" "webots")
for cmd in "${commands[@]}"; do
    if command -v $cmd &> /dev/null; then
        echo "✓ $cmd"
    else
        echo "✗ $cmd"
    fi
done

echo ""
echo "======================================"
echo "Installation complete!"
echo "======================================"