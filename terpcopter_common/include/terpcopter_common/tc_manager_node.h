#pragma once

#include <list>
#include <map>
#include <string>
#include <terpcopter_common/common.h>
#include <terpcopter_common/system.h>
#include <terpcopter_common/tc_node.h>
#include <unistd.h>

class TCManagerNode : public TCNode {
  public:

    //FIXME should be pointer to subscriber, but not available easily
    // in c++03
    std::map<std::string, ros::Subscriber> curr_sys_map;
    std::map<std::string, system_s> sys_map;

    virtual int initialize_systems(std::vector<std::string> &sys_list) = 0;

    int teardown_systems();
    
    // Check system map
    int check_systems();
   
    // Add generic system to sys_map
    // Use args to start system
    int add_system(const system_s &sys);

    // Remove generic system from sys_map
    int remove_system(const std::string &sys_name);

    // Constructor
    TCManagerNode(std::string &nm);
    TCManagerNode(const char *nm);

    // Destructor
    ~TCManagerNode();

  private:
    int sys_idx;

    // Looks for system message/service to update systems list
    void health_sub_cb(const terpcopter_comm::Health::ConstPtr &msg);

    // Start system with start_cmd
    int start_sys(const std::string &sys_name);

    // End system thru end_cmd
    int end_sys(const std::string &sys_name);
};
