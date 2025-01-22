#!/bin/bash

# Navigate to ArduPilot folder
cd ~/eletroquad/ardupilot

# Configure ArduPilot for SITL (Software-in-the-Loop simulator)
if [ ! -f "build/sitl/bin/arducopter" ]; then
    echo "Configuring ArduPilot for SITL..."
    ./waf configure --board sitl
    echo "Building ArduPilot copter..."
    ./waf copter
else
    echo "ArduPilot SITL is already configured and built."
fi

# Return to eletroquad folder
cd ~/eletroquad

# Setup Python virtual environment for custom scripts
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment for custom scripts..."
    python3 -m venv venv
    source venv/bin/activate
    echo "Installing Python dependencies for custom scripts..."
    pip install --upgrade pip
    pip install dronekit pymavlink MAVProxy tensorflow opencv-python-headless yolov5
    deactivate
else
    echo "Virtual environment already exists."
fi

# Download QGroundControl
if [ ! -f "QGroundControl.AppImage" ]; then
    echo "Downloading QGroundControl..."
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.0/QGroundControl.AppImage
    chmod +x QGroundControl.AppImage
else
    echo "QGroundControl is already downloaded and set up."
fi

# Run QGroundControl
echo "Launching QGroundControl..."
./QGroundControl.AppImage
