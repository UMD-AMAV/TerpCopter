#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <terpcopter_common/system.h>

using namespace std;

// Define global threshold time to determine if message is relevant
// Messages seem to take ~.01-.05s to transfer on this system
const ros::Duration msg_threshold(.11);

// -----------------------------------------

// General system functions ----------------
int wrd_list_to_string(std::string &s, char **wrds) {
  // Empty destination string
  s.assign("");
  std::string tmp;
  int wrd_len=0;
  int i=0;
  if (!wrds && wrds[0]) {
    ROS_INFO("no (char *) words available to copy to string");
    return SUCCESS;
  }

  while (wrds[i]) {
    wrd_len=0;
    while(wrds[i][wrd_len++]);
    tmp.assign(wrds[i]);
    s += tmp.assign(wrds[i]);
    s += " ";
    i++;
  }

  return SUCCESS;
}

bool wrd_array_equal(const char *lhs[], const char *rhs[]){
  while (*lhs){
    if (!(*rhs) ||
        strcmp(*lhs++,*rhs++)) {
      return false;
    }
  }

  return true;
}

bool wrd_array_equal(char *const *lhs, char *const *rhs){
  while (*lhs){
    if (!(*rhs) ||
        strcmp(*lhs++,*rhs++)) {
      return false;
    }
  }

  return true;
}

bool operator==(const system_s &lhs, const system_s &rhs) {
  ROS_INFO("damn, we r in system_s == method");
  if (!lhs.name.compare(rhs.name) &&
      !lhs.pkg.compare(rhs.name) &&
      wrd_array_equal(lhs.start_cmd,rhs.start_cmd) &&    
      wrd_array_equal(lhs.end_cmd,rhs.end_cmd) &&    
      (lhs.ros == rhs.ros))
    return true;
  else
    return false;
}

// Handle SIGCHLD
// Useful for tc_manager_node
void child_handler(int sig) {
  pid_t pid;
  union wait wstat;
  int status;

  //FIXME make this more portable
  // Give roskill chance to exit thru system call cleanup before waiting.
  // Does this make sense?
  // It seems to work...
  sleep(1);   

  while(1) {
    pid = wait3(&status, WNOHANG, (struct rusage *)NULL);
    if (!pid)
      return;
    else if (pid == -1)
      return;
    else {
      if (WIFEXITED(status)) {
        ROS_INFO("pid %d: child exited with status code %d\n",
            pid, WEXITSTATUS(status));
      } else if (WIFSIGNALED(status)) {
        ROS_INFO("pid %d: child killed by signal %d\n",
          pid, WTERMSIG(status));
      }
    }
  }

  fflush(stdout);fflush(stderr);
}

// -----------------------------------------

// struct system_s functions ---------------

// Constructors
system_s::system_s() {
  start_cmd=NULL;
  end_cmd=NULL;
}

system_s::system_s(std::string nm, std::string pack,
      char **st_cmd, char **nd_cmd, bool is_ros){
  name.assign(nm);
  pkg.assign(pack);
  ros=is_ros;
  alloc_cmds(st_cmd,nd_cmd);
}

// Deep copy constructor
system_s::system_s(const system_s &sys) {
  name.assign(sys.name);
  pkg.assign(sys.pkg);
  ros=sys.ros;
  alloc_cmds(sys.start_cmd,sys.end_cmd);
}

// system_s deep copy assignment
void system_s::assign(std::string nm, std::string pack,
      char **st_cmd, char **nd_cmd, bool is_ros){
  free_sys(*this);
  name.assign(nm);
  pkg.assign(pack);
  ros=is_ros;
  alloc_cmds(st_cmd,nd_cmd);
}

// Free members that have memory allocated to them 
void system_s::free_sys(system_s &sys) {
  free_cmds(sys.start_cmd,sys.end_cmd);
}

void system_s::free_cmds(
    char **st_cmd, char **nd_cmd) {
  int i=0;
  while (st_cmd && st_cmd[i]) {
    free(st_cmd[i++]);
  }

  free(st_cmd);

  i=0;
  while (nd_cmd && nd_cmd[i]) {
    free(nd_cmd[i++]);
  }

  free(nd_cmd);
}

void system_s::print() const {
  std::string strt_str,end_str;
  wrd_list_to_string(strt_str,start_cmd);
  wrd_list_to_string(end_str,end_cmd);

  //FIXME should be ROS_DEBUG
  ROS_INFO("System:\n\
\tname = %s\n\tpkg = %s\n\tstart_cmd = %s\n\t\
end_cmd = %s\n\tros = %s",
    name.c_str(),pkg.c_str(),
    strt_str.c_str(),end_str.c_str(),
    ros ? "true" : "false");
}

bool system_s::equals(const system_s &cmp) {
  return (this == &cmp);
}


//void system_s::alloc_cmds(const char **st_cmd, const char **nd_cmd) {
void system_s::alloc_cmds(char *const *st_cmd, char *const *nd_cmd) {
  int num_wrds=0;
  while(st_cmd[num_wrds++]);
  start_cmd = (char **) malloc(num_wrds*sizeof(st_cmd) + 1);

  int i=0;
  while(st_cmd[i]) {
    start_cmd[i] = (char *) malloc(strlen(st_cmd[i]) + 1);
    strcpy(start_cmd[i],st_cmd[i]);
    i++;
  }
  start_cmd[i]=NULL;

  num_wrds=0;
  while(nd_cmd[num_wrds++]);
  end_cmd = (char **) malloc(num_wrds*sizeof(nd_cmd) + 1);

  i=0;
  while(nd_cmd[i]) {
    end_cmd[i] = (char *) malloc(strlen(nd_cmd[i]) + 1);
    strcpy(end_cmd[i],nd_cmd[i]);
    i++;
  }
  end_cmd[i]=NULL;
}

// -----------------------------------------
