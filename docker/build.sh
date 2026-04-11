#!/bin/bash

# prepare sources
rosdep update
sudo apt update

# Manual installation (debugging)
sudo apt install pip -y
sudo apt update

# Installing OpenCV
sudo apt install -y libopencv-dev python3-opencv

# fetch repo
git clone https://github.com/ubc-subbots/spiderfish.git
cd spiderfish

# install deps and build
rosdep install -i --from-path src --rosdistro jazzy -y
source /opt/ros/jazzy/setup.bash
colcon build
