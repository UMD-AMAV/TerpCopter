#!/bin/bash

if [ -z ${PX4_FIRMWARE} ]; then
  echo "ERROR - Must add PX4_FIRMWARE env var\
    pointing to <path>/<to>/PX4/Firmware"
  return
fi

if [ -z ${CATKIN_HOME} ]; then
  echo "ERROR - Must add CATKIN_HOME env var\
    pointing to <path>/<to>/catkin_ws"
  return
fi

# debugging
# DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source px4/firmware as prescribed by
# dev.px4.io/simulation-ros-interface.html
echo "Making PX4/Firmware posix_sitl_default target"
make -C ${PX4_FIRMWARE} posix_sitl_default

echo "Sourcing catkin_ws just in case it has not been done"
source ${CATKIN_HOME}/devel/setup.bash

echo "Setting up PX4/Firmware for gazebo"
source ${PX4_FIRMWARE}/Tools/setup_gazebo.bash ${PX4_FIRMWARE} ${PX4_FIRMWARE}/build_posix_sitl_default

case ":$ROS_PACKAGE_PATH:" in
  *":$PX4_FIRMWARE:"*) :;;
  *) export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_FIRMWARE";;
esac

case ":$ROS_PACKAGE_PATH:" in
  *":${PX4_FIRMWARE}/Tools/sitl_gazebo:"*) :;;
  *) export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:${PX4_FIRMWARE}/Tools/sitl_gazebo";;
esac
