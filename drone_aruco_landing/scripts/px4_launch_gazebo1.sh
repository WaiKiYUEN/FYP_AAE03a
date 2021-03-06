#! /bin/bash

# Export the desired gps coordinates
export PX4_HOME_LAT=0
export PX4_HOME_LON=0
export PX4_HOME_ALT=0

# Source the PX4 Firmware directory
if [ -z "$1" ]; then
# change this to a location of your firmware! 
FIRMDIR="/home/hubertleung99/Firmware"
else
FIRMDIR=$1
fi
echo $FIRMDIR

# source ros - remeber to change to your version, e.g. melodic/kinetic
source /opt/ros/melodic/setup.bash
# argument used to browse to your PX4 SITL firmware folder
cd $FIRMDIR

# Needed environment for running SITL
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# Run basic PX4 SITL
echo $world
roslaunch px4 new_posix_sitl2.launch world:=$2 vehicle:=$3 sdf:=$4 
