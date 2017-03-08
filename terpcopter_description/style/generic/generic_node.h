#pragma once

#include <terpcopter_common/Health.h>

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <string>
#include <terpcopter_common/common.h>

#define LOCALIZER_NODE "terpcopter_localizer"

// TODO create variable with localized state

class TerpCopterLocalizer {

  public:

    ros::NodeHandle nh;

    terpcopter_common::Health health;

    ros::Publisher health_pub;

    void health_pub_cb(const ros::TimerEvent&);

  private:
    // Check health of all systems
    int8_t check_health();
};
