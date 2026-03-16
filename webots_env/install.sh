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
    ros-jazzy-topic-tools

# Install Gazebo Sim if not already installed
echo "Checking for Gazebo Sim..."
if ! command -v gz &> /dev/null; then
    echo "Installing Gazebo Sim..."
    sudo apt install -y gz-sim
else
    echo "✓ Gazebo Sim already installed"
fi

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
commands=("gz" "ros2" "xacro")
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