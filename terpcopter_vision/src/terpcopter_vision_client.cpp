//
// THIS FILE CONTAINS A SAMPLE IMPLEMENTATION OF A CLIENT TO 
// VISION SERVICES
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

#include <string>

#include "ros/ros.h"
#include "ros/package.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videostab.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "terpcopter_comm/DetectBoundary.h"

///////////////////////////////////////////
//
//	NAMESPACE
//
///////////////////////////////////////////

using namespace cv;

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terpcopter_vision_client");
  ros::NodeHandle n;
  
  std::string image_path = ros::package::getPath("terpcopter_vision") + "/images/caution_tape.jpg";
  Mat image = imread( image_path, 1 );

  if ( !image.data )
    {
        ROS_INFO("No image data \n");
        return -1;
    }

  //Convert OpenCV Mat to ROS Msg
  sensor_msgs::ImagePtr msg;
  msg  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  msg->header.stamp = ros::Time::now() ;

  //Create a client for detect_boundary service
  ros::ServiceClient client = n.serviceClient<terpcopter_comm::DetectBoundary>("detect_boundary");

  terpcopter_comm::DetectBoundary srv;

  //Send the image as a request to the service
  srv.request.input_image = *msg;

  //Call the service, if its a success display the data of the largest contour
  if(client.call(srv))
  {
	  ROS_INFO("Client : Center X = %d, Center Y = %d, Area = %f",srv.response.center_x_pixel,srv.response.center_y_pixel,srv.response.area);
  }
  else
   {
	ROS_ERROR("Failed to call service detect_boundary");
	return 1;
   }

  return 0;
}	
