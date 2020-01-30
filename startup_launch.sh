#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
sudo mount -o rw,users,umask=000 /dev/sda1 /mnt/pendrive
roslaunch mavros px4.launch fcu_url:=/dev/serial0:921600 &
sleep 10
rosrun collect_data ms4525 &
rosrun collect_data hdc1050 &
rosrun collect_data test_system &
rosrun collect_data test_lidar &
python3 /home/pi/scripts/shutdown.py3 &
