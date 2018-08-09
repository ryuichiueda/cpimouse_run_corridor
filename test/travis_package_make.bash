#!/bin/bash -xve

rsync -av ./ ~/catkin_ws/src/cpimouse_run_corridor/

cd ~/catkin_ws/src/
git clone --depth=1 https://github.com/ryuichiueda/raspimouse_ros_2.git

cd ~/catkin_ws
catkin_make
