#!/bin/bash
workspace=$(pwd)


gnome-terminal -t "follow1" -- bash -c "cd ${workspace}/follow1; source devel/setup.bash && roslaunch arm_control arx51.launch; exec bash;"
sleep 1


