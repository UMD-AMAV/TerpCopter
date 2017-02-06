#pragma once

#include <list>
#include <map>
#include <ros/node_handle.h>
#include <string>
#include <terpcopter_common/common.h>
#include <terpcopter_common/system.h>
#include <unistd.h>

#define MANAGER_NODE "terpcopter_manager"

/*
// For handling custom shutdown
void sigIntHandler(int sig);
*/

class TerpCopterManager {

  public:

    ros::NodeHandle nh;
    terpcopter_common::Health health;
    ros::Publisher health_pub;

    std::map<std::string, ros::Subscriber> sys_map;
    std::map<std::string, std::string> sys_pkg_map;

    void health_pub_cb(const ros::TimerEvent&);

    int initialize_systems(const std::string &sys_list);

    int teardown_systems();
    
    // Check system map
    int check_systems();
    
    // Check health of all systems
    int8_t check_health();
    
    // Helper functions to manage systems_list
    int add_system(const std::string& sys_pkg,
        const std::string &sys_name);

    int remove_system(const std::string &sys_name);

    //Constructor
    TerpCopterManager();

  private:
    int sys_idx;

    // Looks for system message/service to update systems list
    void health_sub_cb(const terpcopter_common::Health::ConstPtr &msg);

    // Check if line is part of systems or is comment/empty line
    int check_line(const std::string &line);

    // Start system thru ros
    int start_sys(const std::string &sys_pkg,
        const std::string &sys_name);

    // End system thru ros
    int end_sys(const std::string &sys_name);

};
