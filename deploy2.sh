#!/usr/bin/env bash
selfie_ip="10.10.1.1"
selfie_name="selfie"
my_ip="192.168.0.116"

echo "selfie password:"
read -s password

export ROS_MASTER_URI=$selfie_ip
export ROS_IP=$my_ip


sshpass -p "$password" ssh "${selfie_name}@${selfie_ip}" rm -r /home/rav/Code/selfie_carolocup2020/src && echo "deleted remote workspace"
sshpass -p "$password" scp -r devel "${selfie_name}@${selfie_ip}":/home/rav/Code/selfie_carolocup2020/src && echo "workspace copied"

echo "copied"

sshpass -p "$password" ssh -t "${selfie_name}@${selfie_ip}" ' 
screen -S selfie_screen  -dm
screen -S selfie_screen -X stuff "cd /home/rav/Code/selfie_carolocup2020\n"

screen -S selfie_screen -X stuff "export ROS_MASTER_URI=10.10.1.1\n"

screen -S selfie_screen -X  stuff "export ROS_IP=10.10.1.1\n"
screen -S selfie_screen -X  stuff "cd /home/rav/Code/selfie_carolocup2020\n"

screen -S selfie_screen -X stuff "catkin_make\n"
screen -S selfie_screen -X stuff "source devel/setup.bash\n"
screen -S selfie_screen -X stuff "roslaunch selfie_launch all_debug_rviz.launch\n"
screen -S selfie_screen -r
'

