//
// THIS FILE CONTAINS A SERVICE TO SWEEP SERVO MOTORS
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

#include "terpcopter_driver/terpcopter_servo_server.h"
#include <thread>

///////////////////////////////////////////
//
//	THREADS
//
///////////////////////////////////////////

void servo_sweep_thread()
{
	ros::Rate loop_rate(10);

	current_position = (min_position+max_position)/2;
	int offset = step_size;

 	while (ros::ok())
  	{
	    	cmd_msg.header.stamp = ros::Time::now();
		cmd_msg.header.frame_id = "servo_link";

		cmd_msg.min_range = min_position;
		cmd_msg.max_range = max_position;
		cmd_msg.field_of_view = max_position - min_position;
		cmd_msg.radiation_type = 1 ;

		if(sweep_flag)
		{			
			if( current_position <= min_position)
			{
				offset = step_size;
			}
			else if( current_position >= max_position)
			{
				offset = -step_size;
			}

			current_position += offset;	
		}
	
		else
		{
			current_position = (min_position+max_position)/2;		
		}

		cmd_msg.range = current_position ;
			    
		servo_pub.publish(cmd_msg);

		ros::spinOnce();

		loop_rate.sleep();
  	}

}

///////////////////////////////////////////
//
//	ROS SERVICES
//
///////////////////////////////////////////

bool servo_sweep_service(terpcopter_comm::ServoSweep::Request  &request,
         terpcopter_comm::ServoSweep::Response &response)
{

	response.current_position = current_position;
	sweep_flag = request.flag;
	min_position = request.min_position;
	max_position = request.max_position;
	step_size = request.step_size;

	return true;
}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main(int argc, char **argv)
{

  ros::init(argc, argv, "terpcopter_servo_server");

  ros::NodeHandle n;

  servo_service = n.advertiseService("servo_sweep", servo_sweep_service);

  servo_pub = n.advertise<sensor_msgs::Range>("terpcopter/servo", 1);

  ROS_INFO("Ready to start Servo ");

  std::thread servo_sweep(servo_sweep_thread);

  servo_sweep.join();

  return 0;
}

