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
#include <vector>

#define MANAGER_NODE "terpcopter_manager"

class TerpCopterManager : public TCManagerNode {

  public:
    virtual int initialize_systems(
        const std::vector<std::string> &sys_list);
    
    //Constructor
    TerpCopterManager(std::string &nm);
    TerpCopterManager(const char *nm);

  private:
    // Check if line is part of systems or is comment/empty line
    int check_line(const std::string &line);

    // Check health of all systems
    virtual int8_t check_health();
};
