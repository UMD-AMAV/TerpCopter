//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS OF 
// TERPCOPTER SERVO NODE
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

#ifndef TERPCOPTER_SERVO_SERVER_H_
#define TERPCOPTER_SERVO_SERVER_H_

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <string>

#include "ros/ros.h"
#include "terpcopter_comm/Servo.h"


///////////////////////////////////////////
//
//	NAMESPACES
//
///////////////////////////////////////////

using namespace std;

///////////////////////////////////////////
//
//	ROS VARIABLES
//
/////////////////////////////////////////

sensor_msgs::Range cmd_msg;
ros::ServiceServer servo_service;
ros::Publisher servo_publisher;

///////////////////////////////////////////
//
//	ROS SERVICE DECLARATIONS
//
///////////////////////////////////////////

bool servo(terpcopter_comm::Servo::Request  &request,
         terpcopter_comm::Servo::Response &response) ;



#endif // TERPCOPTER_SERVO_SERVER_H_
