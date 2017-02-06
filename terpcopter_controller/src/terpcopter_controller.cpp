#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <terpcopter_controller.h>

using namespace std;

mavros_msgs::State current_mavros_state;

void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_mavros_state = *msg;
}
