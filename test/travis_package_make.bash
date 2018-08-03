#!/bin/bash -xve

#sync and make
rsync -av ./ ~/catkin_ws/src/cpimouse_run_corridor/

#clone raspimouse_ros_2
cd ~/catkin_ws/src/
git clone https://github.com/citueda/raspimouse_ros_2.git

cd ~/catkin_ws
catkin_make
