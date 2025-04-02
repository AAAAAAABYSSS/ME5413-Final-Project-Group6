#!/bin/bash

# Check and install ultralytic
pip show ultralytics > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "[INFO] Installing Ultralytics YOLO..."
    pip install ultralytics
else
    echo "[INFO] Ultralytics YOLO is already installed."
fi
# # Start roscore
# gnome-terminal --tab -- bash -c "roscore; exec bash"
# sleep 5  # Wait for roscore to initialize

# Build workspace
gnome-terminal --tab -- bash -c "catkin_make -j4; source devel/setup.bash; roslaunch me5413_world world.launch;exec bash"
sleep 10

# Define options
options=("Manual Control Only" "Mapping Mode" "Navigation Mode" "Perception Mode" "Exit")

echo ""
echo "========= Mode Selection ========="
for i in "${!options[@]}"; do
    echo "[$i] ${options[$i]}"
done
echo "=================================="

# Prompt user
read -p "Enter the number corresponding to the mode: " choice
sleep 2
case $choice in
    0)
        echo "Launching manual control only..."
        gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_world manual.launch; exec bash"
        ;;
    1)
        echo "Launching mapping mode..."
        gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_mapping livo2.launch; exec bash"
        ;;
    2)
        echo "Launching navigation mode..."
        gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_perception me5413_perception.launch; exec bash"
        gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_navigation navigation.launch ; exec bash"
        ;;
    3)
        echo "Launching perception mode..."
        gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch me5413_perception me5413_perception.launch; exec bash"
        ;;
    4)
        echo "Exiting script."
        exit 0
        ;;
    *)
        echo "Invalid option. Please run the script again and select a valid number."
        ;;
esac
