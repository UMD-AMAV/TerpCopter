#pragma once

#include <terpcopter_comm/Health.h>

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <string>
#include <terpcopter_common/common.h>
#include <terpcopter_common/tc_node.h>

#define LOCALIZER_NODE "terpcopter_localizer"

// TODO create variable with localized state

class TerpCopterLocalizer : public TCNode {

  public:
    int tmp;

    // Constructors
    TerpCopterLocalizer(std::string &nm);
    TerpCopterLocalizer(const char *nm);
    
  private:

    // Check health of all systems
    virtual int8_t check_health();
};
