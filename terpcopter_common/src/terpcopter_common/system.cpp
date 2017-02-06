#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <stdlib.h>
#include <terpcopter_common/system.h>

using namespace std;

// Define global threshold time to determine if message is relevant
// Messages seem to take ~.01-.05s to transfer on this system
const ros::Duration msg_threshold(.11);
