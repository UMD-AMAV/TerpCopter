#pragma once

#include <terpcopter_comm/Health.h>

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <string>
#include <terpcopter_common/common.h>
#include <terpcopter_common/tc_node.h>

#define CONTROLLER_NODE "terpcopter_controller"

class TerpCopterController : public TCNode {

  public:
    // Messages
    mavros_msgs::State current_mavros_state;
    geometry_msgs::PoseStamped local_pose;

    //Publishers
    ros::Publisher local_pos_pub;

    // Callbacks
    void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void local_pos_pub_cb(const ros::TimerEvent&);

    // Constructors
    TerpCopterController(std::string &nm);
    TerpCopterController(const char *nm);

  private:
    
    // Check health of all systems
    virtual int8_t check_health();
};

