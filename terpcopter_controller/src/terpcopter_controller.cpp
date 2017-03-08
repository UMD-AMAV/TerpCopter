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
    controller.nh.advertise<terpcopter_comm::Health>
    ((std::string(CONTROLLER_NODE) + "_health").c_str(), 100);

  controller.local_pos_pub =
    controller.nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);

  ros::Subscriber mavros_state_sub = 
    controller.nh.subscribe<mavros_msgs::State>
          ("mavros/state", 10, &TerpCopterController::mavros_state_cb,
           &controller);

  ros::ServiceClient arming_client =
    controller.nh.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");

  ros::ServiceClient set_mode_client =
    controller.nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  // The setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(50.0);

  // Create timers
  ros::Timer health_timer = controller.nh.createTimer(
      ros::Duration(.1), &TCNode::health_pub_cb,
      dynamic_cast<TCNode *> (&controller));

  ros::Timer local_pos_timer = controller.nh.createTimer(
      ros::Duration(.02), &TerpCopterController::local_pos_pub_cb,
      &controller);

  // Stop the local_pos timer to do some setup
  local_pos_timer.stop();
 
  // wait for FCU connection
  while(!ros::isShuttingDown() &&
      controller.current_mavros_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  //send a few setpoints before starting
  for(int i = 100; !ros::isShuttingDown() && i > 0; --i){
      controller.local_pos_pub.publish(controller.local_pose);
      ros::spinOnce();
      rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  // Restart local_pos timer
  local_pos_timer.start();

  while(!ros::isShuttingDown()){
    if( controller.current_mavros_state.mode != "OFFBOARD" &&
      (ros::Time::now() - last_request > ros::Duration(5.0)) ){
      if( set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.success){
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !controller.current_mavros_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){

        if( arming_client.call(arm_cmd) &&
          arm_cmd.response.success ){
          ROS_INFO("Vehicle armed");
        }

        last_request = ros::Time::now();
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

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

void TerpCopterController::local_pos_pub_cb(const ros::TimerEvent&) {
  local_pose.pose.position.x = 0;
  local_pose.pose.position.y = 1;
  local_pose.pose.position.z = 2;

  local_pos_pub.publish(local_pose);
}
