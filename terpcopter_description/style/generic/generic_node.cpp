// Perform localization based off of visual and inertial data

#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <signal.h>
#include <string>
#include <terpcopter_common/system.h>
#include <terpcopter_localizer.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, LOCALIZER_NODE);

  // Set logger level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  TerpCopterLocalizer localizer;
  char localizer_node_param[256];

  localizer.nh.setParam(
    strcat(strcat(strcpy(localizer_node_param, "/"),
    LOCALIZER_NODE),"_running"), true);

  ROS_INFO("Started %s node", LOCALIZER_NODE);

  // Define initial list of messages, actions, services
  localizer.health.system = LOCALIZER_NODE;
  localizer.health_pub =
    localizer.nh.advertise<terpcopter_common::Health>
    ((std::string(LOCALIZER_NODE) + "_health").c_str(), 100);

  // Spin at 50Hz
  ros::Rate rate(50.0);

  // Create timers
  ros::Timer health_timer = localizer.nh.createTimer(
      ros::Duration(.1), &TerpCopterLocalizer::health_pub_cb, &localizer);
 
  while(!ros::isShuttingDown()){
    ros::spinOnce();
    rate.sleep();
  }

  //FIXME Cleanup section not running correctly
  // Cleanup

  ROS_INFO("here");
  fflush(stdout);
  localizer.nh.setParam(
    strcat(strcat(strcpy(localizer_node_param, "/"),
    LOCALIZER_NODE),"_running"), false);

  return SUCCESS;
}

//TODO
// Check health of all systems
int8_t TerpCopterLocalizer::check_health() {
//DEBUG
#include <cstdlib>
  int ran = std::rand() % 100;
  // Return ERROR on 5% of cases
  if (ran >= 5 )
    return SUCCESS;
  else
    return RESTART_SYS;
}

void TerpCopterLocalizer::health_pub_cb(const ros::TimerEvent&) {
    health.health = check_health();
    health.t = ros::Time::now();
    health_pub.publish(health);
}
