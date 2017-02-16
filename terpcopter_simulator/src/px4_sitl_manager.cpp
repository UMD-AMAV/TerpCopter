// Start PX4 SITL manager
// Start PX4 Gazebo SITL TODO add support for other simulators
// Check health from all running systems
// Shutdown PX4 Gazebo SITL

#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <px4_sitl_manager.h> 
using namespace std;

PX4SITLManager *manager=NULL;

int main(int argc, char **argv) {
  ros::init(argc, argv, MANAGER_NODE, ros::init_options::NoSigintHandler);
  //TODO read simulator argument into sys_list
  
  
  // Set logger level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  manager = new PX4SITLManager(MANAGER_NODE);
  
  char manager_node_param[256];

  manager->nh.setParam(
    strcat(strcpy(manager_node_param,MANAGER_NODE),"_running"),
    true);

  // Define initial list of messages, actions, services
  manager->health.system = MANAGER_NODE;
  manager->health_pub = manager->nh.advertise<terpcopter_common::Health>
    ((std::string(MANAGER_NODE) + "_health").c_str(), 100);

  // Spin at 50Hz
  ros::Rate rate(50.0);

  // Create timers
  ros::Timer health_timer = manager->nh.createTimer(
      ros::Duration(.1), &TCNode::health_pub_cb,
      dynamic_cast<TCNode *> (manager));

  // initialize systems from systems file 
  manager->initialize_systems(sys_list);
  sleep(1);

  manager->check_systems();

  ROS_DEBUG("Entering %s main loop",MANAGER_NODE);
  while(!ros::isShuttingDown()){
    ros::spinOnce();
    rate.sleep();
  }

  manager->teardown_systems();
  delete manager;

  return SUCCESS;
}

// Constructors
TerpCopterManager::TerpCopterManager(std::string &nm) :
  TCManagerNode(nm)
{
}

TerpCopterManager::TerpCopterManager(const char *nm) :
  TCManagerNode(nm)
{
}

// Start systems and subscribe to health of all nodes listed
// Health messages will be named health_system_name
int TerpCopterManager::initialize_systems(
    const std::string &sys_list) {

    // Start simulator

    if (add_system(sys_pkg,sys_name) < 0) {
      ROS_ERROR("Cannot add system %s %s",
        sys_pkg.c_str(), sys_name.c_str());
      return ERROR;
    }
  }

  return SUCCESS;
}

// Return SUCCESS if not comment line (starts with #)
// Return ERROR if comment line
int TerpCopterManager::check_line(const std::string &line) {
  const string delims(" \t\n");
  std::wstring::size_type pos = line.find_first_not_of(delims);

  if (line[0] == DESC_COMMENT_CHAR ||
      pos == string::npos) {
    return ERROR;
  }

  return SUCCESS;
}

// Check health of all systems
int8_t TerpCopterManager::check_health() {
  return SUCCESS;
}
