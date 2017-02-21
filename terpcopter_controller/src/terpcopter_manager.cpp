// Start terpcopter manager
// Add systems to systems_list from terpcopter_systems.txt
// Check health from all running systems
// Execute mission plan from mission_plan.txt
// Shutdown terpcopter

#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <terpcopter_manager.h> 
using namespace std;

TerpCopterManager *manager=NULL;

int main(int argc, char **argv) {
  ros::init(argc, argv, MANAGER_NODE, ros::init_options::NoSigintHandler);
  // Set logger level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  manager = new TerpCopterManager(MANAGER_NODE);
  
  char manager_node_param[256];

  manager->nh.setParam(
    strcat(strcpy(manager_node_param,MANAGER_NODE),"_running"),
    true);

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

  // read mission_plan.txt and terpcopter_systems.txt
  string terpcopter_mission_path = ros::package::getPath("terpcopter_description") + "/mission/";
  string terpcopter_systems_file = terpcopter_mission_path+MISSION_SYSTEMS_FILE;
  string terpcopter_mission_file = terpcopter_mission_path+MISSION_PLAN_FILE;

  // initialize systems from systems file 
  vector<string> v;
  v.push_back(terpcopter_systems_file);
  manager->initialize_systems(v);
  sleep(1);

  manager->check_systems();
   ROS_DEBUG("Entering %s main loop",MANAGER_NODE);
  while(!ros::isShuttingDown()){
    ros::spinOnce();
    rate.sleep();
  }

  //FIXME not running correctly b/c ROS kills connection with master
  // during shutdown
  // For now, ROS backend stuff will handle teardown
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
    const std::vector<std::string> &sys_list) {
  ifstream sys_file(sys_list.front().c_str());
  string line, sys_pkg, sys_name, delim = " ";
  size_t pos = 0;
  int i;

  ROS_DEBUG("Initializing systems from %s manager", name.c_str());
  while (getline(sys_file, line)) {
    // Check if line is part of systems or is comment/empty line
    if (check_line(line) < 0) continue;

    for (i=0; i<2; i++) {
      pos = line.find(delim);

      switch (i) {
        case 0:
          sys_pkg = line.substr(0,pos);
          break;
        case 1:
          sys_name = line.substr(0,pos);
          break;
        default:
          ROS_ERROR("Attempting to find too many parameters for mission system");
      }
      
      line.erase(0,pos+delim.length());
    }

    // FIXME Dirty dirty conversions to get rid of casting errors
    char nm[256],pack[256],sys_name_ns[256];
    strcpy(nm,sys_name.c_str());
    strcpy(pack,sys_pkg.c_str());
    memset(&sys_name_ns[0],0,
        sizeof(sys_name_ns)/sizeof(sys_name_ns[0]));
    strcat(strcat(sys_name_ns,"/"),nm);

    char *start_cmd[] = {"rosrun",pack,nm,NULL};
    char *end_cmd[] = {"rosnode","kill",sys_name_ns,NULL};

    // Correct way to start a system
    // Must instantiate system
    // Then call add_system()
    // Then call system_s::free_sys() once it is put in the map
    system_s sys(sys_name,sys_pkg,
        start_cmd, end_cmd,
        true);

    if (add_system(sys) < 0) {
      ROS_ERROR("Cannot add system %s %s",
        sys_pkg.c_str(), sys_name.c_str());
      return ERROR;
    }

    system_s::free_sys(sys);
    sys_map.find(sys_name)->second.print();
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
