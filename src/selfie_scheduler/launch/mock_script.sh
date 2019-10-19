#!/bin/bash

read -p "Press enter to press start button"
rostopic pub --once /start_button std_msgs/Bool "data: false"
echo -e "\e[32mStart button pressed \033[0m"

read -p "Press enter to start moving the car"
rostopic pub --once /distance std_msgs/Float32 "data: 0.5" 
echo -e "\e[32mCar started to move \033[0m"

read -p "Press enter to move car given distance"
rostopic pub --once /distance std_msgs/Float32 "data: 3.0"
echo -e "\e[32mCar covered distance \033[0m"

read -p "Press enter to detect start line"
rostopic pub --once /starting_line std_msgs/Float32 "data: 0.0"
echo -e "\e[32mStart line detected \033[0m"

echo "end script"
