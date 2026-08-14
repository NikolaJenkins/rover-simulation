# Project Goal

Build an autonomous rover simulation using ROS 2 and Gazebo.

## Features:
- Differential drive rover
- Camera
- Lidar
- Obstacle avoidance
- Future SLAM support

# Steps to replicate

Run 
```
git clone git@github.com:NikolaJenkins/rover-simulation.git && cd ~/rover-simulation
```
Build the container with
```
podman build -t rover-dev ~/rover-simulation
```
Build the image with
```
distrobox create \
    --name rover-dev-container \
    --image rover-dev
```
