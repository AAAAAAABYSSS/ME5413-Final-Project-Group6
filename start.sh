gnome-terminal --tab -- bash -c "roscore; exec bash"
sleep 5  # Wait for roscore to initialize

# Build
gnome-terminal --tab -- bash -c "catkin_make -j4;source devel/setup.bash; exec bash"

sleep 16

# Launch Gazebo World together with our robot
gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_world world.launch; exec bash"

sleep 5

# # Only launch the robot keyboard teleop control
gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_world manual.launch; exec bash"
# gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_mapping livo2.launch; exec bash"


