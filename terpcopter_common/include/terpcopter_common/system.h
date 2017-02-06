#pragma once

#include <terpcopter_common/Health.h>

#include <string>
#include <map>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <terpcopter_common/common.h>
#include <unistd.h>

// Define mission files
#define MISSION_SYSTEMS_FILE "terpcopter_systems.txt"
#define MISSION_PLAN_FILE "mission_plan.txt"

// Comment character for mission files
#define DESC_COMMENT_CHAR '#'

// Define parent system
#define PARENT_SYS "terpcopter_manager"

// Define health parameters
#define HEALTHY 0
#define RESTART_SYS -1
#define STOP_SYS -2

struct system_s {
  std::string name;
  ros::Subscriber sub;
};

// Global message threshold time to determine if message is relevant
extern const ros::Duration msg_threshold;
