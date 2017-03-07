#include <terpcopter_common/tc_node.h>

// Constructor
TCNode::TCNode(std::string &nm) :
  name(nm)
{
}

// Constructor
TCNode::TCNode(const char *nm) :
  name(nm)
{
}

void TCNode::health_pub_cb(const ros::TimerEvent&) {
  health.health = check_health();
  health.t = ros::Time::now();
  health_pub.publish(health);
}

int8_t TCNode::check_health() {
  return SUCCESS;
}
