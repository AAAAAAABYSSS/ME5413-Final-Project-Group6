#!/bin/bash

gnome-terminal --tab -- bash -c "
  cd /home/jy/fun_project;
  roslaunch me5413_world world.launch;
  exec bash
"


sleep 3



# Launch cartographer in a new terminal
gnome-terminal --tab -- bash -c "
  cd /home/jy/fun_project;
  source /home/jy/fun_project/ME5413_Final_Project/src/third_party/ros_motion_planning/devel/setup.bash;
  roslaunch jackal_navigation navigation2.launch;
  exec bash
"
echo "Running"