//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS OF 
// THE KEYBOARD TELEOP CLASS
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

#ifndef TERPCOPTER_KEYBOARD_TELEOP_NODE_H_
#define TERPCOPTER_KEYBOARD_TELEOP_NODE_H_


///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>

///////////////////////////////////////////
//
//	DEFINITIONS
//
///////////////////////////////////////////

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_UA 0x41
#define KEYCODE_DA 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64

///////////////////////////////////////////
//
//	NAMESPACE
//
///////////////////////////////////////////

using namespace geometry_msgs;
using namespace std;

///////////////////////////////////////////
//
//	VARIABLES
//
///////////////////////////////////////////

int kfd = 0;
struct termios cooked, raw;

///////////////////////////////////////////
//
//	CLASSES
//
///////////////////////////////////////////


class KeyboardTeleop
 {

 private:   
   ros::NodeHandle nh;
   double roll, pitch, yaw_rate, thrust;
   double thrust_offset,roll_offset,pitch_offset,yaw_rate_offset;
   ros::Publisher cmd_vel_pub;
   geometry_msgs::Twist cmd_vel_msg;
   bool publish_flag;
   char c;

 public:
   KeyboardTeleop(string topic_name);
   void Run(); 
   
 };

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

void quit(int sig);


#endif // TERPCOPTER_KEYBOARD_TELEOP_NODE_H_
