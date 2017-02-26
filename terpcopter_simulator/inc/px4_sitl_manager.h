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
#define MANAGER_NODE_PKG "terpcopter_simulator"
#define SIMULATOR "px4_sitl"
#define SIMULATOR_PKG "px4/Firmware"

#define SIM_START_SCRIPT "px4_sitl_start.sh"
#define SIM_END_SCRIPT "px4_sitl_end.sh"

class PX4SITLManager : public TCManagerNode {

  public:
    virtual int initialize_systems(std::vector<std::string> &sys_list);
    
    //Constructor
    PX4SITLManager(std::string &nm);
    PX4SITLManager(const char *nm);
    
  private:
    // Check health of all systems
    virtual int8_t check_health();
};
