//
// THIS FILE CONTAINS THE IMPLEMENTATION FOR ESTIMATING THE POSITION OF 
// THE DRONE BY INTEGRATING PX4FLOW'S GROUND VELOCITY ESTIMATE
//
// COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE
//
// AUTHOR : LAKSHMAN KUMAR
// AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
// EMAIL : LKUMAR93@TERPMAIL.UMD.EDU
// LINKEDIN : WWW.LINKEDIN.COM/IN/LAKSHMANKUMAR1993
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THE GPLv3 LICENSE
// THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF
// THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS 
// PROHIBITED.
// 
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <terpcopter_sensor_processor/terpcopter_localization_node.h>

///////////////////////////////////////////
//
//	VARIABLES
//
///////////////////////////////////////////

bool initialized = false;

float position_z_lpf_threshold = 15;

double dt = 0.0;
double position_x = 0.0;
double position_y = 0.0;
double position_z= 0.0;
double prev_position_z = 0.0;
double prev_time = 0.0;

///////////////////////////////////////////
//
//	SUBSCRIBER FUNCTIONS
//
///////////////////////////////////////////

void Integrate(const terpcopter_comm::OpticalFlow::ConstPtr& px4flow_packet)
{

 if(initialized == true)
{

// Compute time interval between current and previous packet

 dt = px4flow_packet->header.stamp.toSec() - prev_time;

// Integrate ground velocities from Optical Flow sensor to get the position

 position_x = position_x  + (px4flow_packet->velocity_x*dt);
 position_y = position_y  + (px4flow_packet->velocity_y*dt);
 position_z = px4flow_packet->ground_distance;
 
// Eliminate noise from PX4Flow's Ultrasonic sensor's height reading 

 if((position_z - prev_position_z)/dt > position_z_lpf_threshold)
	position_z = prev_position_z;

// Publish position to the topic "terpcopter/position"

 position_msg.point.x = position_x;
 position_msg.point.y = position_y;
 position_msg.point.z = position_z;
 position_msg.header.stamp = ros::Time::now();
 position_msg.header.frame_id = "world";
 
 position_pub.publish(position_msg);

// Publish Linear Velocity to the topic "terpcopter/linear_velocity"

 linear_velocity_msg.point.x = px4flow_packet->velocity_x;
 linear_velocity_msg.point.y = px4flow_packet->velocity_y;
 linear_velocity_msg.point.z = (position_z - prev_position_z)/dt;
 linear_velocity_msg.header.stamp = ros::Time::now();
 linear_velocity_msg.header.frame_id = "world";

 linear_velocity_pub.publish(linear_velocity_msg);

}

// Set Previous Values to current values to make use of in the next iteration

 prev_time = px4flow_packet->header.stamp.toSec();
 prev_position_z = px4flow_packet->ground_distance;

 if(initialized == false)
 	initialized = true;

}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terpcopter_localization_node");

  ros::NodeHandle n;

  optical_flow_sub = n.subscribe("terpcopter/cameras/bottom/optical_flow", 10, Integrate);

  position_pub = n.advertise<geometry_msgs::PointStamped>("terpcopter/position", 10);
  linear_velocity_pub = n.advertise<geometry_msgs::PointStamped>("terpcopter/linear_velocity", 10);

  ros::spin();

  return 0;
}

