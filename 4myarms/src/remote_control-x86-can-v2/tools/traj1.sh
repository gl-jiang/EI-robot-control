#!/bin/bash
workspace=~/4myarms/
sleep 1
gnome-terminal -t "follow1" -- bash -c "cd ${workspace}/follow1; source devel/setup.bash && roslaunch arm_control arx5.launch; exec bash;"
sleep 1
