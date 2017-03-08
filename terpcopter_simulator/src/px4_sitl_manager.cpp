// Start PX4 SITL manager
// Start PX4 Gazebo SITL TODO add support for other simulators
// Check health from all running systems
// Shutdown PX4 Gazebo SITL

#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <cstring>
#include <terpcopter_simulator/px4_sitl_manager.h> 
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
  ROS_INFO("Started %s node", MANAGER_NODE); 

  // Define initial list of messages, actions, services
  manager->health.system = MANAGER_NODE;
  manager->health_pub = manager->nh.advertise<terpcopter_comm::Health>
    ((std::string(MANAGER_NODE) + "_health").c_str(), 100);

  // Spin at 50Hz
  ros::Rate rate(50.0);

  // Create timers
  ros::Timer health_timer = manager->nh.createTimer(
      ros::Duration(.1), &TCNode::health_pub_cb,
      dynamic_cast<TCNode *> (manager));

  // Define systems to initialize
  vector<string> v;
  v.push_back("px4_sitl");
  v.push_back("px4_sitl_manager");
  manager->initialize_systems(v);
  ROS_INFO("here");
  sleep(1);

  manager->check_systems();

  ROS_INFO("Entering %s main loop",MANAGER_NODE);
  while(!ros::isShuttingDown()){
    ros::spinOnce();
    rate.sleep();
  }

  manager->teardown_systems();
  delete manager;

  return SUCCESS;
}

// Constructors
PX4SITLManager::PX4SITLManager(std::string &nm) :
  TCManagerNode(nm)
{
}

PX4SITLManager::PX4SITLManager(const char *nm) :
  TCManagerNode(nm)
{
}

// Start systems and subscribe to health of all nodes listed
// Health messages will be named health_system_name
int PX4SITLManager::initialize_systems(
    std::vector<std::string> &sys_list) {
  //FIXME these conversions are too dirty
  system_s sys;
  string sys_pkg, sys_name;
  char nm[256],pack[256],sys_name_ns[256], tc_tools[MAX_PATH_LEN];
  bool ros;
  ROS_DEBUG("Initializing %s systems",name.c_str());
  
  // Assign system properties
  while (!sys_list.empty()) {
    sys_name = sys_list.back();
    sys_list.pop_back();
    strcpy(nm,sys_name.c_str());
    memset(&sys_name_ns[0],0,
        sizeof(sys_name_ns)/sizeof(sys_name_ns[0]));
    strcat(strcat(sys_name_ns,"/"),nm);

    ROS_DEBUG("Initializing system %s",sys_name.c_str());

    if (!strcmp(nm,MANAGER_NODE)) {
      sys_pkg=MANAGER_NODE;
      strcpy(pack,MANAGER_NODE_PKG);
      char *start_cmd[] = {"rosrun",pack,nm,NULL};
      char *end_cmd[] = {"rosnode","kill",sys_name_ns,NULL};
      ros = true;
      sys.assign(sys_name, sys_pkg,
          start_cmd, end_cmd, ros);
    } else if (!strcmp(nm,SIMULATOR)) {
      sprintf(tc_tools,"%s",secure_getenv("TERPCOPTER_TOOLS"));
      if (!strcmp(tc_tools,"")) {
        ROS_ERROR("Missing TERPCOPTER_TOOLS environment variable");
        return ERROR;
      }

      char start_script[MAX_PATH_LEN], end_script[MAX_PATH_LEN];
      strcat(strcat(start_script,tc_tools),SIM_START_SCRIPT);
      strcat(strcat(end_script,tc_tools),SIM_END_SCRIPT);

      sys_pkg=SIMULATOR_PKG;
      strcpy(pack,SIMULATOR_PKG);
      char *start_cmd[] = {start_script, NULL};
      char *end_cmd[] = {end_script, NULL};
      string tmp;
      wrd_list_to_string(tmp,start_cmd);
      ROS_INFO("start cmd: %s",tmp.c_str());
      wrd_list_to_string(tmp,end_cmd);
      ROS_INFO("end cmd: %s",tmp.c_str());
      ros = false;
      sys.assign(sys_name, sys_pkg,
          start_cmd, end_cmd, ros);
    }

    if (add_system(sys) < 0) {
      ROS_ERROR("Cannot add system %s %s",
        sys_pkg.c_str(), sys_name.c_str());
      return ERROR;
    }
  }

  system_s::free_sys(sys);

  return SUCCESS;
}

// Check health of all systems
// TODO extend to work with non-ROS systems
int8_t PX4SITLManager::check_health() {
  return SUCCESS;
}
