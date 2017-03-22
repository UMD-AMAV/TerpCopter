//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES OF 
// THE TERPCOPTER_OBSTACLE_DETECTION_NODE
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

#ifndef TERPCOPTER_OBSTACLE_DETECTION_NODE_H_
#define TERPCOPTER_OBSTACLE_DETECTION_NODE_H_

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

///////////////////////////////////////////
//
//	NAMESPACES
//
///////////////////////////////////////////

using namespace sensor_msgs;
using namespace message_filters;

///////////////////////////////////////////
//
//	DEFINITIONS
//
///////////////////////////////////////////

#define DISTANCE_THRESHOLD 1.0 //in m

typedef sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range> ObstacleDetectionSyncPolicy;

///////////////////////////////////////////
//
//	VARIABLES
//
///////////////////////////////////////////

int current_servo_angle = 75;
float current_range_value = 0;
int field_of_view = 50;

float cumulative_right_range_value = 0.0;
float cumulative_left_range_value = 0.0;
float cumulative_front_range_value = 0.0;

int count = 0;
int left_count = 0;
int front_count = 0;
int right_count = 0;

int split_factor;
float left_limit;
float right_limit;

float average_right_range_value = 0.0;
float average_left_range_value = 0.0;
float average_front_range_value = 0.0;

ros::Publisher obstacle_detection_pub;

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

void obstacle_detection(const Range::ConstPtr& Lidarmsg, const Range::ConstPtr& Servomsg);


#endif // TERPCOPTER_OBSTACLE_DETECTION_NODE_H_
