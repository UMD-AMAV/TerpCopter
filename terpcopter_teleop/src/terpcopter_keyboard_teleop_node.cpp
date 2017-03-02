//
// THIS FILE CONTAINS THE IMPLEMENTATION FOR TELE-OPERATING TERPCOPTER USING A 
// KEYBOARD
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

#include "terpcopter_teleop/terpcopter_keyboard_teleop_node.h"

#include <signal.h>
#include <termios.h>
#include <stdio.h>

///////////////////////////////////////////
//
//	CLASS METHODS
//
///////////////////////////////////////////

KeyboardTeleop::KeyboardTeleop(string topic_name):
  roll(0.0),
  pitch(0.0),
  yaw_rate(0.0),
  thrust(0.0),
  roll_offset(1.0),
  pitch_offset(1.0),
  yaw_rate_offset(1.0),
  thrust_offset(1000.0),
  publish_flag(false)
{
  cmd_vel_pub =  nh.advertise<geometry_msgs::Twist>(topic_name, 1);
}

void KeyboardTeleop::Run()
{

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);  

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move terpcopter.");
  puts("Up Arrow = Increase Pitch, Down Arrow = Decrease Pitch");
  puts("Left Arrow = Decrease Roll, Right Arrow = Increase Roll");
  puts("W = Increase Thrust, S = Decrease Thrust");
  puts("A = Decrease Yaw Rate, D = Increase Yaw Rate");
  puts("Q = Stop");
  puts("---------------------------");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      ROS_INFO("Keyboard Read Error");
      exit(-1);
    }

   ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("Decrease Roll");
        roll -= roll_offset;
        ROS_INFO("%f",roll);
	publish_flag = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("Increase Roll");
        roll += roll_offset;
	publish_flag = true;
        break;
      case KEYCODE_UA:
        ROS_DEBUG("Increase Pitch");
        pitch += pitch_offset;
	publish_flag = true;
        break;
      case KEYCODE_DA:
        ROS_DEBUG("Decrease Pitch");
        pitch -= pitch_offset;
	publish_flag = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("Decrease Yaw Rate");
        yaw_rate -= yaw_rate_offset;
	publish_flag = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("Increase Yaw Rate");
        yaw_rate += yaw_rate_offset;
	publish_flag = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("Increase Thrust");
        thrust += thrust_offset;
	publish_flag = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("Decrease Thrust");
        thrust -= thrust_offset;
	publish_flag = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("STOP");
        thrust = 0.0;
	roll = 0.0;
	pitch = 0.0;
        yaw_rate = 0.0;
        publish_flag = true;
        break;
    }
   
    cmd_vel_msg.linear.z = thrust;
    cmd_vel_msg.linear.x = pitch;
    cmd_vel_msg.linear.y = roll;
    cmd_vel_msg.angular.y = yaw_rate;

    if(publish_flag == true)
    {
      cmd_vel_pub.publish(cmd_vel_msg);    
      publish_flag=false;
      ROS_INFO("Thrust = %f, Roll = %f ,Pitch = %f, Yaw Rate = %f",thrust,roll,pitch,yaw_rate);
    }
  }
  return;
}

///////////////////////////////////////////
//
//	HELPER FUNCTION
//
///////////////////////////////////////////

void quit(int sig)
{
   tcsetattr(kfd, TCSANOW, &cooked);
   ros::shutdown();
   exit(0);
}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "terpcopter_keyboard_teleop_node");

  KeyboardTeleop teleop_terpcopter("terpcopter/cmd_vel");

  signal(SIGINT,quit);

  teleop_terpcopter.Run();
  
  return(0);
}

