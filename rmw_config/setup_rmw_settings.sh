#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#Declaring the path to rmw settings
export FASTRTPS_DEFAULT_PROFILES_FILE=$SCRIPT_DIR'/rmw_settings.xml'

#adding to bashrc file file too
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE='$SCRIPT_DIR'/rmw_settings.xml' >> ~/.bashrc

#Restarting ROS daemon for the settings to take effect
ros2 daemon stop

echo 'RMW settings setup complete.'
