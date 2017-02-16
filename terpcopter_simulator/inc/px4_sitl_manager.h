#pragma once

#include <terpcopter_common/Health.h>

#include <list>
#include <map>
#include <ros/node_handle.h>
#include <string>
#include <terpcopter_common/common.h>
#include <terpcopter_common/system.h>
#include <terpcopter_common/tc_manager_node.h>
#include <unistd.h>

#define MANAGER_NODE "px4_sitl_manager"

class PX4SITLManager : public TCManagerNode {

  public:
    virtual int initialize_systems(const std::string &sys_list);
    
    //Constructor
    PX4SITLManager(std::string &nm);
    PX4SITLManager(const char *nm);
    
    // Add system to sys_map using args to start system
    int add_system(const std::string& sys_pkg,
        const std::string &sys_name,
        const char *args[]);

    // Remove system from sys_map using args to end system
    int remove_system(const std::string &sys_name,
        const char *args[]);

  private:
    // Check health of all systems
    virtual int8_t check_health();

    // Start simulator
    int start_
};
