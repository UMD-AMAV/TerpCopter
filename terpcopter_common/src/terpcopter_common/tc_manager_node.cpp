#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <terpcopter_common/tc_manager_node.h>

// Constructor
TCManagerNode::TCManagerNode(std::string &nm) :
  TCNode(nm), sys_idx(0)
{
}

TCManagerNode::TCManagerNode(const char *nm) :
  TCNode(nm), sys_idx(0)
{
}

int TCManagerNode::teardown_systems() {
  int ret;
  // Check sys_idx > 1 to avoid re-trying to remove terpcopter_controller
  ROS_INFO("Tearing down %s member systems", name.c_str());
  for (std::map<std::string, ros::Subscriber>::iterator it=curr_sys_map.begin();
      it!=curr_sys_map.end() && sys_idx > 1; ++it) {
    ret = remove_system(it->first);
    if (ret < 0 && it->first.compare(name)) {
      ROS_ERROR("Failed to remove system %s from TerpCopter systems",
        it->first.c_str());
      return ERROR;
    } else if (ret < 0) {
      ROS_ERROR("Cannot remove system %s, as it is the parent system",
          it->first.c_str());
    }
  }

  return SUCCESS;
}

// Check system map
// TODO make display in numerical order, currently displays in weird
// tree order
int TCManagerNode::check_systems() {
  ROS_INFO("List of current systems:");
  for (std::map<std::string, ros::Subscriber>::iterator it=curr_sys_map.begin();
      it!=curr_sys_map.end(); ++it) {
    ROS_INFO("\tSystem %s",
        it->first.c_str());
  }
 
  return SUCCESS;
}

// Looks for system message/service to update systems list
void TCManagerNode::health_sub_cb(
  const terpcopter_common::Health::ConstPtr &msg) {
  // Determine healthy system or not
  if (ros::Time::now() - msg->t < msg_threshold) {
    switch(msg->health) {
      case HEALTHY:
        // Do nothing
        break;
      case STOP_SYS:
        ROS_INFO("%s stopping system %s",
            name.c_str(),msg->system.c_str());
        if (msg->system.compare(name)) {
          remove_system(msg->system);
        } else { // Managing sys
          ROS_ERROR("Stopping current managing system %s",
              name.c_str());
        }
        break;
      case RESTART_SYS:
        ROS_INFO("%s restarting system %s",
            name.c_str(),msg->system.c_str());
        if (msg->system.compare(name) &&
            !remove_system(msg->system)) {
          if (sys_map.find(msg->system) != sys_map.end()) {
            add_system(sys_map.find(msg->system)->second);
          } else {
            ROS_ERROR("Can't find %s in %s system map",
                msg->system.c_str(), name.c_str());
          }
        } else { // Managing system
          ROS_INFO("Restarting current managing system %s",
              name.c_str());
        }

        break;
      default:
        break;
    }
  }
}

// Add generic system to sys_map
// Use args to start system
int TCManagerNode::add_system(const system_s &sys) {
  int sys_ret;
  bool b;
  // Check if system is in map
  // If in map, return SUCCESS
  // Else check if system exists and is running
  // If exists and is not running, launch and add to map
  // Else if exists and is running, add to map
  if (curr_sys_map.find(sys.name) == curr_sys_map.end()) {
    ROS_INFO("Adding system %s %s to TerpCopter systems",
        sys.name.c_str(), sys.pkg.c_str());
    
    if (sys_map.find(sys.name) == sys_map.end()) {
      sys_map.insert(std::pair<std::string, system_s>(
        sys.name, sys));
      ROS_INFO("Inserted %s %s to sys_map",
        sys.name.c_str(), sys.pkg.c_str());
    } else if (!(sys_map.find(sys.name) == sys_map.end()) &&
        !(sys_map.find(sys.name)->second.equals(sys))) {
      ROS_INFO("Replacing old system %s in %s system map",
          sys.name.c_str(), name.c_str());
      system_s::free_sys(sys_map[sys.name]);
      sys_map[sys.name] = sys;
    }

    if (nh.hasParam("/"+sys.name+"_running")) {
      nh.getParam("/"+sys.name+"_running", b);

      if (!b) { // Restarting system 
        sys_ret = start_sys(sys.name);
        sleep(1);
        nh.getParam("/"+sys.name+"_running", b);
        if (!sys_ret && b) {
          ROS_DEBUG("Successfully restarted system %s",
            sys.name.c_str());
        } else {
          ROS_ERROR("Error restarting system %s", sys.name.c_str());
          return ERROR;
        }
      } else {
        ROS_DEBUG("System %s already running", sys.name.c_str());
      }

    } else { // Starting system from scratch
      sys_ret = start_sys(sys.name);
      sleep(1);
      nh.getParam("/"+sys.name+"_running", b);

      if (!sys_ret && b) {
        ROS_DEBUG("Successfully started system %s",
          sys.name.c_str());
      } else {
        ROS_ERROR("Error starting system %s", sys.name.c_str());
          return ERROR;
      }
    }

    if (sys.ros) {
      curr_sys_map.insert(std::pair<std::string, ros::Subscriber>(sys.name,
        nh.subscribe(
          sys.name + "_health", 100,
          &TCManagerNode::health_sub_cb, this)));
    } else {
      curr_sys_map.insert(std::pair<std::string, ros::Subscriber>(sys.name,
      ros::Subscriber()));
    }

    // increment system index
    sys_idx++;
  } else {
    ROS_WARN("%s already in TerpCopter systems", sys.name.c_str());
    return ERROR;
  }
  
  return SUCCESS;
}

// Remove ROS node from sys_map 
int TCManagerNode::remove_system(const std::string &sys_name) {
  int sys_ret;
  bool b;

  if (curr_sys_map.find(sys_name) != curr_sys_map.end()) {
    if (nh.hasParam("/"+sys_name+"_running")) {
      nh.getParam("/"+sys_name+"_running",b);
      if (b && !sys_name.compare(name)) {
        ROS_ERROR("Cannot kill %s this way,\
as it is the managing system",
          sys_name.c_str());
        return ERROR;
      } else if (b) {
        ROS_INFO("Killing system %s from %s",
            sys_name.c_str(),name.c_str());
        sys_ret = end_sys(sys_name);
        if (!sys_ret) {
          // Set system to no longer be running
          // Must set here because shutdown detaches node from master
          nh.setParam(("/"+sys_name+"_running"), false);
          ROS_INFO("Successfully killed system %s", sys_name.c_str()); 
        } else {
          ROS_ERROR("Error killing system %s", sys_name.c_str());
          return ERROR;
        }
      } else {
        ROS_DEBUG("System to be killed, %s, not running"
            ,sys_name.c_str());
      }
    } else {
      ROS_DEBUG("System to be killed, %s, not running"
          ,sys_name.c_str());
    }

    curr_sys_map.erase(sys_name);
    if (--sys_idx < 0) {
      ROS_ERROR("Number of systems < 0");
      return ERROR;
    }
  } else {
    ROS_WARN("Cannot remove %s: not in TerpCopter systems",
        sys_name.c_str());
    return ERROR;
  }

  return SUCCESS;
}

// Start system
int TCManagerNode::start_sys(const std::string &sys_name) {
  if (sys_map.find(sys_name) != sys_map.end()) {
    system_s *sys = &(sys_map.find(sys_name)->second);
    pid_t pid;

    pid = fork();
    if (pid == 0) {
      ROS_INFO("Child process pid %u\nSystem %s %s",
          getpid(),
          sys->pkg.c_str(),
          sys->name.c_str());
      fflush(stdout); fflush(stderr);

      //FIXME
      // 1. Problem: execvp() may succeed even though node doesn't start
      //    in the case that start_cmd (eg rosnode) executes correctly
      // 2. Handle failure better
      execvp(sys->start_cmd[0],sys->start_cmd);

      ROS_ERROR("Could not start system %s %s",
        sys->pkg.c_str(), sys->name.c_str());

      exit(EXIT_FAILURE);

    } else if (pid > 0) {
      ROS_DEBUG("Started child process pid %u\n\
for system %s %s",
          pid,
          sys->pkg.c_str(), sys->name.c_str());
      
    } else {
      return ERROR;
    }
  
    return SUCCESS;
  } else {
    ROS_ERROR("Can't find system %s in sys_map",sys_name.c_str());
    return ERROR;
  }
}

//TODO make this a threaded thing
// End system
int TCManagerNode::end_sys(const std::string &sys_name) {
  int sys_ret;
  std::string s;
  if (sys_map.find(sys_name) != sys_map.end()) {
    wrd_list_to_string(s, sys_map.find(sys_name)->second.end_cmd);
    ROS_INFO("Ending system %s with command %s",
        sys_name.c_str(), s.c_str());

    sys_ret = system(s.c_str());
    if (sys_ret < 0) {
      ROS_ERROR("Unable to end system %s",
        sys_name.c_str());
      return ERROR;
    }
  } else {
    ROS_INFO("Can't find system %s in %s system map",
        sys_name.c_str(), name.c_str());
    return ERROR;
  }

  return SUCCESS;
}

