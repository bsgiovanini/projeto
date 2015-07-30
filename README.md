
.bashrc

source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/workshop
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/workshop/tum_simulator
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_workspace/src
export octomap_DIR=~/octomap/octomap


para rodar:

roslaunch projeto room.launch
