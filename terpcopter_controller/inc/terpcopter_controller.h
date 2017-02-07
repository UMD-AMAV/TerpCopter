#pragma once

#include <terpcopter_common/Health.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <string>
#include <terpcopter_common/common.h>

#define CONTROLLER_NODE "terpcopter_controller"

class TerpCopterController {

  public:

    ros::NodeHandle nh;

    // Messages
    terpcopter_common::Health health;
    mavros_msgs::State current_mavros_state;

    // Publishers
    ros::Publisher health_pub;

    // Callbacks
    void health_pub_cb(const ros::TimerEvent&);
    void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);

  private:
    // Check health of all systems
    int8_t check_health();
};

