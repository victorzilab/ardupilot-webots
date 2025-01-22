#!/bin/bash

# Creating base folder
mkdir -p ~/eletroquad
cd ~/eletroquad

# Installing system dependencies
echo "Checking and installing system dependencies..."
declare -a packages=("python3.12-venv" "libgl1-mesa-dev" "libxkbcommon-x11-0" "libqt5gui5" "qt5-qpa-platformplugin" "git" "gitk" "git-gui" "python3-pip")
for package in "${packages[@]}"; do
    if ! dpkg -l | grep -qw "$package"; then
        echo "Installing $package..."
        sudo apt install -y "$package"
    else
        echo "$package is already installed."
    fi
done

# Installing Webots
if [ ! -d "webots" ]; then
    echo "Downloading and setting up Webots..."
    wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b-x86-64.tar.bz2
    tar -xvf webots-R2023b-x86-64.tar.bz2
    rm webots-R2023b-x86-64.tar.bz2
else
    echo "Webots is already set up."
fi

# Cloning ArduPilot repository
if [ ! -d "ardupilot" ]; then
    echo "Cloning ArduPilot repository..."
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    . ~/.profile
else
    echo "ArduPilot repository is already cloned."
fi

echo "Setup before logout is complete. Please reboot your system."
