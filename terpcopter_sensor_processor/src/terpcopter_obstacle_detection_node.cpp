//
// THIS FILE CONTAINS THE IMPLEMENTATION FOR DETECTING OBSTACLES
// BASED ON THE DATA FROM TERA RANGER ONE AND SERVO MOTOR
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

#include <terpcopter_sensor_processor/terpcopter_obstacle_detection_node.h>

#include <terpcopter_comm/ObstacleDetection.h>

///////////////////////////////////////////
//
//	SUBSCRIBER FUNCTIONS
//
///////////////////////////////////////////

void obstacle_detection(const Range::ConstPtr& tera_ranger_msg, const Range::ConstPtr& servo_msg)
{

	terpcopter_comm::ObstacleDetection obst_detect_msg;

  	current_servo_angle = servo_msg->range;
  	current_range_value = tera_ranger_msg->range;
	field_of_view = servo_msg->field_of_view;
	split_factor = field_of_view/3.0;
	right_limit = servo_msg->min_range + split_factor; 
	left_limit = servo_msg->max_range - split_factor; 
	  
  	if(current_servo_angle < right_limit)
  	{
   		cumulative_right_range_value += current_range_value;  
   		right_count++;
  	}
   
  	else if(current_servo_angle > left_limit)
  	{
   		cumulative_left_range_value += current_range_value;
   		left_count++;
        }

  	else
   	{
     		cumulative_front_range_value += current_range_value;
     		front_count++;
   	}
  
  	count = front_count + left_count + right_count;
  
  	if( count >= 2*field_of_view )
  	{
  
  		average_right_range_value = (cumulative_right_range_value)/right_count;
  		average_left_range_value = (cumulative_left_range_value)/left_count;
  		average_front_range_value = (cumulative_front_range_value)/front_count;
  
	  	if(average_right_range_value <= DISTANCE_THRESHOLD)
	  	{
	  		obst_detect_msg.is_obstacle_right = true;
  
	  	}
	  	else
	  	{
	  		obst_detect_msg.is_obstacle_right = false;
	  	}
  
	  	if(average_left_range_value <= DISTANCE_THRESHOLD)
	  	{
	  		obst_detect_msg.is_obstacle_left = true;
  	  	}
	        else
	  	{
	  		obst_detect_msg.is_obstacle_left = false;
	  	}
  
	  	if(average_front_range_value <= DISTANCE_THRESHOLD)
	  	{
	  		obst_detect_msg.is_obstacle_front = true;
		}
	  	else
	  	{
	  		obst_detect_msg.is_obstacle_front = false;
	  	}

		ROS_INFO("Average Right = %f , Average Front = %f, Average Left = %f",average_right_range_value,average_front_range_value,average_left_range_value);
  
  		obst_detect_msg.stamp = ros::Time::now();
		obst_detect_msg.range = tera_ranger_msg->range;
		obst_detect_msg.angle = servo_msg->range;
		obst_detect_msg.distance_threshold = DISTANCE_THRESHOLD;
		
  
  		obstacle_detection_pub.publish(obst_detect_msg); 
    
  		front_count = 0;
  		left_count = 0;
  		right_count = 0;
  
  		cumulative_right_range_value = 0.0;
  		cumulative_left_range_value = 0.0;
		cumulative_front_range_value = 0.0;
	}	

}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terpcopter_obstacle_detection_node");

  ros::NodeHandle n;

  obstacle_detection_pub = n.advertise<terpcopter_comm::ObstacleDetection>("terpcopter/obstacle_detection", 10);

  message_filters::Subscriber<sensor_msgs::Range> tera_ranger_sub(n, "terpcopter/teraranger/range", 5);
  message_filters::Subscriber<sensor_msgs::Range> servo_sub(n, "terpcopter/servo", 3);

  Synchronizer<ObstacleDetectionSyncPolicy> sync(ObstacleDetectionSyncPolicy(10),tera_ranger_sub,servo_sub);

  sync.registerCallback(boost::bind(&obstacle_detection,_1,_2));

  ros::spin();

  return 0;
}

