#pragma once

#include <string>
#include <vector>
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
//#define PARENT_SYS "terpcopter_manager"

// Define health parameters
#define HEALTHY 0
#define RESTART_SYS -1
#define STOP_SYS -2

// Define max number of commands allowed for a start or end command
#define MAX_WRD_LEN 80 

// FIXME this is so dirty
// Define max length of path
#define MAX_PATH_LEN 1024

struct system_s {
public:
  std::string name;
  std::string pkg;
  char **start_cmd=NULL;
  char **end_cmd=NULL;
  bool ros;

  // Constructors
  system_s();
  system_s(std::string nm, std::string pack,
      char **st_cmd, char **nd_cmd, bool is_ros);
  system_s(const system_s &sys);

  void assign(std::string nm, std::string pack,
      char **st_cmd, char **nd_cmd, bool is_ros);

  static void free_sys(system_s &sys);
  static void free_cmds(char **st_cmd, char **nd_cmd);
  bool equals(const system_s &cmp);
  void print() const;
  
private:
  void alloc_cmds(char *const *st_cmd, char *const *nd_cmd);
};

int wrd_list_to_string(std::string &s, char **wrds);

bool wrd_array_equal(char const **lhs, char const **rhs);
bool wrd_array_equal(char *const *lhs, char *const *rhs);

bool operator==(const system_s &lhs, const system_s &rhs);

void child_handler(int sig);

// Global message threshold time to determine if message is relevant
extern const ros::Duration msg_threshold;
