// Start terpcopter manager
// Add systems to systems_list from terpcopter_systems.txt
// Check health from all running systems
// Execute mission plan from mission_plan.txt
// Shutdown terpcopter

#include <terpcopter_common/Health.h>

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

  manager = new TerpCopterManager();
  
  char manager_node_param[256];

  manager->nh.setParam(
    strcat(strcpy(manager_node_param,MANAGER_NODE),"_running"),
    true);

  /* Necessary any more?
  // Define custom sigint handler
  signal(SIGINT, sigIntHandler);
  */

  // Define initial list of messages, actions, services
  manager->health.system = MANAGER_NODE;
  manager->health_pub = manager->nh.advertise<terpcopter_common::Health>
    ((std::string(MANAGER_NODE) + "_health").c_str(), 100);

  // Spin at 50Hz
  ros::Rate rate(50.0);

  // Create timers
  ros::Timer health_timer = manager->nh.createTimer(
      ros::Duration(.1), &TerpCopterManager::health_pub_cb, manager);

  // read mission_plan.txt and terpcopter_systems.txt
  string terpcopter_mission_path = ros::package::getPath("terpcopter_description") + "/mission/";
  string terpcopter_systems_file = terpcopter_mission_path+MISSION_SYSTEMS_FILE;
  string terpcopter_mission_file = terpcopter_mission_path+MISSION_PLAN_FILE;

  // initialize systems from systems file 
  manager->initialize_systems(terpcopter_systems_file);
  sleep(1);

  manager->check_systems();
  
  //DEBUG
  manager->remove_system("terpcopter_localizer");
  manager->add_system("terpcopter_sensor_processor",
      "terpcopter_localizer");

  ROS_DEBUG("Entering %s main loop",MANAGER_NODE);
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

  //FIXME not running, not sure why
  manager->teardown_systems();
  delete manager;

  return SUCCESS;
}

// Constructor
TerpCopterManager::TerpCopterManager(void) {
  sys_idx = 0;
}

/*
void sigIntHandler(int sig) {
  manager->teardown_systems();

  delete manager;
  ros::shutdown();
}
*/

// Start systems and subscribe to health of all nodes listed
// Health messages will be named health_system_name
int TerpCopterManager::initialize_systems(
    const std::string &sys_list) {
  ifstream sys_file(sys_list.c_str());
  string line, sys_pkg,
         sys_name, delim = " ";
  size_t pos = 0;
  int i;

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

    //TODO subscribe to system health message

    if (add_system(sys_pkg,sys_name) < 0) {
      ROS_ERROR("Cannot add system %s %s",
        sys_pkg.c_str(), sys_name.c_str());
      return ERROR;
    }
  }

  return SUCCESS;
}

int TerpCopterManager::teardown_systems() {
  int ret;
  // Check sys_idx > 1 to avoid re-trying to remove terpcopter_controller
  ROS_INFO("Tearing down TerpCopter systems");
  for (std::map<std::string, ros::Subscriber>::iterator it=sys_map.begin(); it!=sys_map.end() && sys_idx > 1; ++it) {
    ret = remove_system(it->first);
    if (ret < 0 && it->first.compare(PARENT_SYS)) {
      ROS_ERROR("Failed to remove system %s from TerpCopter systems",
        it->first.c_str());
      return ERROR;
    } else if (ret < 0) {
      ROS_ERROR("Cannot remove system %s, as it is the parent system",
          it->first.c_str());
    }
    //TODO unsubscribe from system health message
    //TODO set system_running params to 0
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

// Check system map
// TODO make display in numerical order, currently displays in weird
// tree order
int TerpCopterManager::check_systems() {
  ROS_INFO("List of current systems:");
  for (std::map<std::string, ros::Subscriber>::iterator it=sys_map.begin(); it!=sys_map.end(); ++it) {
    ROS_INFO("\tSystem %s",
        it->first.c_str());
  }
 
  return SUCCESS;
}

// Check health of all systems
int8_t TerpCopterManager::check_health() {
  return SUCCESS;
}

// Looks for system message/service to update systems list
// TODO stop or restart unhealthy systems
void TerpCopterManager::health_sub_cb(
    const terpcopter_common::Health::ConstPtr &msg) {
  // Determine healthy system or not
  if (ros::Time::now() - msg->t < msg_threshold) {
    switch(msg->health) {
      case HEALTHY:
        // Do nothing
        break;
      case STOP_SYS:
        ROS_DEBUG("Stopping system %s",msg->system.c_str());
        if (msg->system.compare(PARENT_SYS)) {
          remove_system(msg->system);
        } else { // parent sys

        }
        break;
      case RESTART_SYS:
        ROS_DEBUG("Restarting system %s",msg->system.c_str());
        if (msg->system.compare(PARENT_SYS)
            && !remove_system(msg->system)) {
          add_system(sys_pkg_map[msg->system], msg->system);
        } else { // parent sys

        }
        break;
      default:
        break;
    }
  }
}

// Add system to system map
// Cannot add same node twice, but ROS does allow for anonymously named nodes
// Thus, multiple of the same node can run with different names
// TODO add in ability to pass params
// TODO add in ability to start node in namespace
//      Maybe do this thru launch files
int TerpCopterManager::add_system(const std::string &sys_pkg,
    const std::string &sys_name) {
  int sys_ret;
  bool b;
  // Check if system is in map
  // If in map, return SUCCESS
  // Else check if system exists and is running
  // If exists and is not running, launch and add to map
  // Else if exists and is running, add to map
  if (sys_map.find(sys_name) == sys_map.end()) {
    if (nh.hasParam("/"+sys_name+"_running")) {
      nh.getParam("/"+sys_name+"_running", b);

      if (!b) { // Restarting system 
        sys_ret = start_sys(sys_pkg, sys_name);
        sleep(1);
        nh.getParam("/"+sys_name+"_running", b);
        if (!sys_ret && b) {
          ROS_DEBUG("Successfully restarted system %s",
            sys_name.c_str());
        } else {
          ROS_ERROR("Error restarting system %s", sys_name.c_str());
          return ERROR;
        }
      } else {
        ROS_DEBUG("System %s already running", sys_name.c_str());
      }

    } else { // Starting system from scratch
      sys_ret = start_sys(sys_pkg, sys_name);

      sleep(1);
      nh.getParam("/"+sys_name+"_running", b);

      if (!sys_ret && b) {
        ROS_DEBUG("Successfully started system %s",
          sys_name.c_str());
      } else {
        ROS_ERROR("Error starting system %s", sys_name.c_str());
          return ERROR;
      }
    }

    ROS_INFO("Adding system %s to TerpCopter systems",
        sys_name.c_str());

    sys_map.insert(std::pair<std::string, ros::Subscriber>(sys_name,
      nh.subscribe(
        sys_name + "_health", 100,
        &TerpCopterManager::health_sub_cb, this)));

    // increment system index
    sys_idx++;

    if (sys_pkg_map.find(sys_name) == sys_pkg_map.end()) {
      sys_pkg_map.insert(std::pair<std::string, std::string>(
        sys_name, sys_pkg));
      ROS_DEBUG("inserted %s %s",
        sys_pkg_map[sys_name].c_str(), sys_name.c_str());
    }

  } else {
    ROS_WARN("%s already in TerpCopter systems", sys_name.c_str());
    return ERROR;
  }
  
  return SUCCESS;
}

// Remove system from system map
int TerpCopterManager::remove_system(const std::string &sys_name) {
  int sys_ret;
  bool b;
  // Check if system is in map and is running
  // If in map, remove
  // If not in map, return ERROR
  if (sys_map.find(sys_name) != sys_map.end()) {
    // TODO check if node is running
    // TODO kill node
    if (nh.hasParam("/"+sys_name+"_running")) {
      nh.getParam("/"+sys_name+"_running", b);

      if (b && !sys_name.compare(PARENT_SYS)) {
        ROS_ERROR("Cannot kill %s this way, as it is the parent system",
          sys_name.c_str());
        return ERROR;
      } else if (b) {
        sys_ret = end_sys(sys_name);
        if (!sys_ret) {

          //FIXME take this out once shutdown section of nodes is workign
          // Set system to no longer be running
          nh.setParam(("/"+sys_name+"_running"), false);
          
          ROS_DEBUG("Successfully killed system %s", sys_name.c_str()); 
        } else {
          ROS_ERROR("Error killing system %s", sys_name.c_str());
          return ERROR;
        }
      } else {
        ROS_DEBUG("System %s not running",sys_name.c_str());
      }
    }

    ROS_INFO("Removing %s from TerpCopter systems",
        sys_name.c_str());
    sys_map.erase(sys_name);
    
    // Don't erase node names so that they can be restarted
    // sys_pkg_map.erase(sys_name);

    if (--sys_idx < 0) {
      ROS_ERROR("Number of systems < 0");
      return ERROR;
    }

  } else {
    ROS_WARN("Cannot remove %s: not in TerpCopter systems", sys_name.c_str());
    return ERROR;
  }

  return SUCCESS;
}

// Start system thru ros
int TerpCopterManager::start_sys(const std::string &sys_pkg,
    const std::string &sys_name) {
    pid_t pid;

    fflush(stdout);
    fflush(stderr);

    pid = fork();
    if (pid == 0) {
      ROS_DEBUG("Child process pid %u\nSystem %s %s",
          getpid(),
          sys_pkg.c_str(), sys_name.c_str());

      //TODO clean this up
      char pkg[1024], nm[1024];
      strcpy(pkg, sys_pkg.c_str());
      strcpy(nm, sys_name.c_str());
      char *args[] = {"rosrun", pkg,
                      nm, NULL};

      execvp(args[0],args);

      ROS_ERROR("Could not start system %s %s",
        sys_pkg.c_str(), sys_name.c_str());

      exit(EXIT_FAILURE);

    } else if (pid > 0) {
      ROS_DEBUG("Started child process pid %u\n\
for system %s %s",
          pid,
          sys_pkg.c_str(), sys_name.c_str());
      
    } else {
      return ERROR;
    }
  
    return SUCCESS;
}

// End system thru ros
int TerpCopterManager::end_sys(const std::string &sys_name) {
  int sys_ret;

  sys_ret = system(("rosnode kill /" + sys_name).c_str());
  if (sys_ret < 0) {
    ROS_ERROR("Unable to end system %s",
      sys_name.c_str());
    return ERROR;
  }
  
  return SUCCESS;
}

void TerpCopterManager::health_pub_cb(const ros::TimerEvent&) {
    health.health = check_health();
    health.t = ros::Time::now();
    health_pub.publish(health);
}
