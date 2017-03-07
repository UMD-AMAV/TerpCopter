#!/bin/bash
if [ "$PX4_FIRMWARE" == "" ] || [ "$PX4_FIRMWARE" == "none" ]
then
	echo "Missing PX4_FIRMWARE environment varable"
  exit 1
fi

cd $PX4_FIRMWARE
xterm -e make posix_sitl_default gazebo

exit 0
