# Project Goal

Build an autonomous rover simulation using ROS 2 and Gazebo.

## Features:
- Differential drive rover
- Camera
- Lidar
- Obstacle avoidance
- Future SLAM support

# Steps to replicate
Create a new folder with
```
mkdir ~/ROS && cd ~/ROS
```
Run 
```
git clone git@github.com:NikolaJenkins/rover-simulation.git && cd ~/ROS/rover-simulation
```
Build the image with
```
podman build -t rover-dev ~/ROS/rover-simulation
```
Build the container with
```
distrobox create --name rover-dev-container --image rover-dev
```
Enter the container with
```
distrobox enter rover-dev-container
```
Follow the steps in 'Install TurtleBot3 Packages' section at this link: https://docs.robotis.com/docs/systems/turtlebot3/quick_start_guide/pc_setup/.

Follow the steps in this link to setup the Gazebo simulation: https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation/?ros=jazzy#install-simulation-package.
