//
// THIS FILE CONTAINS A SERVICE TO CONTROL SWEEP SERVO MOTORS
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

#include "ros/ros.h"
#include "terpcopter_comm/ServoSweep.h"

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////


int main(int argc, char **argv)
{

  ros::init(argc, argv, "terpcopter_servo_client");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<terpcopter_comm::ServoSweep>("servo_sweep");

  terpcopter_comm::ServoSweep srv;

  srv.request.flag = true;
  srv.request.max_position = 125;
  srv.request.min_position = 75;
  srv.request.step_size = 1;

  if(client.call(srv))
  {
	ROS_INFO("Servo Client : Current Position = %d",srv.response.current_position);
  }
  else
  {
	ROS_ERROR("Failed to call service servo_sweep");
	return 1;

  }

  return 0;
}

