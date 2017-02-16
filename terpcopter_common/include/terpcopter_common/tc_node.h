#pragma once

#include <terpcopter_common/Health.h>

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <terpcopter_common/common.h>

class TCNode {

  public:

    ros::NodeHandle nh;

    terpcopter_common::Health health;

    ros::Publisher health_pub;

    std::string name;

    void health_pub_cb(const ros::TimerEvent&);

    //Constructors
    TCNode(std::string &nm);
    TCNode(const char *nm);

  private:
    // Check health of all systems
    virtual int8_t check_health();
};
