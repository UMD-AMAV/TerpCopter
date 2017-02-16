#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <terpcopter_common/system.h>
#include <terpcopter_controller.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, CONTROLLER_NODE);

  // Set logger level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  // Converting constructor
  TerpCopterController controller(CONTROLLER_NODE);
  char controller_node_param[256];

  controller.nh.setParam(
    strcat(strcat(strcpy(controller_node_param, "/"),
    CONTROLLER_NODE),"_running"), true);

  ROS_INFO("Started %s node", CONTROLLER_NODE);

  // Define initial list of messages, actions, services
  controller.health.system = CONTROLLER_NODE;
  controller.health_pub =
    controller.nh.advertise<terpcopter_common::Health>
    ((std::string(CONTROLLER_NODE) + "_health").c_str(), 100);

  // Spin at 50Hz
  ros::Rate rate(50.0);

  // Create timers
  ros::Timer health_timer = controller.nh.createTimer(
      ros::Duration(.1), &TCNode::health_pub_cb,
      dynamic_cast<TCNode *> (&controller));
 
  while(!ros::isShuttingDown()){
    ros::spinOnce();
    rate.sleep();
  }

  /* FIXME add after ensuring health works well
  ros::Subscriber mavros_state_sub = nh.subscribe<mavros_msgs::State>
          ("mavros/state", 10, mavros_state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && current_mavros_state.connected){
      ros::spinOnce();
      rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 1;
  pose.pose.position.z = 2;

  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while(ros::ok()){
      if( current_mavros_state.mode != "OFFBOARD" &&
          (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( set_mode_client.call(offb_set_mode) &&
              offb_set_mode.response.success){
              ROS_INFO("Offboard enabled");
          }
          last_request = ros::Time::now();
      } else {
          if( !current_mavros_state.armed &&
              (ros::Time::now() - last_request > ros::Duration(5.0))){
              if( arming_client.call(arm_cmd) &&
                  arm_cmd.response.success){
                  ROS_INFO("Vehicle armed");
              }
              last_request = ros::Time::now();
          }
      }

      local_pos_pub.publish(pose);

      ros::spinOnce();
      rate.sleep();
  }

  */

  // Cleanup

  return SUCCESS;
}

// Constructors
TerpCopterController::TerpCopterController(std::string &nm) :
  TCNode(nm)
{
}

TerpCopterController::TerpCopterController(const char *nm) :
  TCNode(nm)
{
}

//TODO
// Check health of all systems
int8_t TerpCopterController::check_health() {
  return SUCCESS;
}

void TerpCopterController::mavros_state_cb(
    const mavros_msgs::State::ConstPtr& msg){
  current_mavros_state = *msg;
}
