//
// THIS FILE CONTAINS VARIOUS VISION SERVICES
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

#include "terpcopter_vision/terpcopter_vision_server.h"

#include "ros/package.h"

///////////////////////////////////////////
//
//	HELPER FUNCTIONS
//
///////////////////////////////////////////

// Detect a particular color based on the corresponding H , S , V values
Mat detect_color(const Mat& input_image, int LOW_H, int LOW_S, int LOW_V,int HIGH_H, int HIGH_S, int HIGH_V)
{
  Mat thresholded_image;

  inRange(input_image, Scalar( LOW_H, LOW_S, LOW_V), Scalar(HIGH_H, HIGH_S, HIGH_V), thresholded_image); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
  erode(thresholded_image, thresholded_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( thresholded_image, thresholded_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( thresholded_image, thresholded_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(thresholded_image, thresholded_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  
  return thresholded_image;

}

//Combine two images together and saturate them between the min and max limits

Mat combine_images(const Mat& input_image1,const Mat& input_image2)
{
    Mat combined_image = input_image1.clone();

    for(int j = 0; j < input_image1.rows; j++)
	for(int i = 0; i < input_image1.cols; i++)
	{
	    //High pass filter the image by subtracting input_image from low pass filtered image and saturate the output
	    combined_image.at<uchar>(j,i) = saturate_cast<uchar>(input_image1.at<uchar>(j,i) + input_image2.at<uchar>(j,i));
	}

  return combined_image;
}

// Find the largest contour in the given image
ContourDataT find_contours(const Mat& input_image)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  vector<Point> largest_contour;
  int largest_contour_id;
  float largest_contour_length;
  float largest_contour_area;
  int largest_contour_center_x;
  int largest_contour_center_y;
  int number_of_contours;

  ContourDataT contour_data;

  //Random Number Generator
  RNG rng(12345);


  //Find all the contours in the image
  findContours( input_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
 
 /// Draw contours
  Mat image_of_contours = Mat::zeros( input_image.size(), CV_8UC3 );

  for( int i = 0; i< contours.size(); i++ )
  {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( image_of_contours , contours, i, color, 2, 8, hierarchy, 0, Point() );
  }
   

  number_of_contours = contours.size();

  
  if (number_of_contours > 0 )
  {

	  float previous_area = 0.0;
	  float current_area = 0.0;

	  // Find contour with largest area
	  for( int i = 0; i < contours.size(); i++ )
	     { 
		current_area = contourArea(contours[i]);
	
		if ( current_area > previous_area )
		{
			largest_contour_area = current_area;
			largest_contour_id = i;	
			previous_area = current_area;	
		}

	     }

	   largest_contour = contours[largest_contour_id];
	   largest_contour_length = arcLength(largest_contour , true );

	    /// Get the moments of the largest contour
	   Moments mu = moments( largest_contour, false );

	   largest_contour_center_x = mu.m10/mu.m00 ;
	   largest_contour_center_y = mu.m01/mu.m00;
	 
	   contour_data.image_of_contours = image_of_contours;
	   contour_data.center_x = largest_contour_center_x;
	   contour_data.center_y = largest_contour_center_y;
	   contour_data.area = largest_contour_area;
	   contour_data.arc_length = largest_contour_length;
	   contour_data.number_of_contours = number_of_contours;

   }

   else
   {
	 contour_data.image_of_contours = image_of_contours;
	 contour_data.center_x = 0;
	 contour_data.center_y = 0;
	 contour_data.area = 0;
	 contour_data.arc_length = 0;
	 contour_data.number_of_contours = 0;
   }
   
   ROS_INFO("Server : Center X = %d, Center Y = %d, Area = %f",largest_contour_center_x,largest_contour_center_y,largest_contour_area);

   return contour_data;

}


///////////////////////////////////////////
//
//	ROS SERVICES
//
///////////////////////////////////////////

bool detect_boundary(terpcopter_comm::DetectBoundary::Request  &request,
         terpcopter_comm::DetectBoundary::Response &response)
{

  cv_bridge::CvImagePtr image_ptr;
  Mat input_image, hsv_image, yellow_image, black_image, combined_image,detected_image;

  ContourDataT largest_contour_data;
  try
  {
    image_ptr = cv_bridge::toCvCopy( request.input_image , sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  input_image = image_ptr->image;
 
  //Convert to HSV Format
  cvtColor(input_image, hsv_image, COLOR_BGR2HSV);

  //Detect Yellow color
  yellow_image = detect_color(hsv_image, LOW_H_YELLOW, LOW_S_YELLOW, LOW_V_YELLOW, HIGH_H_YELLOW, HIGH_S_YELLOW, HIGH_V_YELLOW );

  //Detect Black color
  black_image = detect_color(hsv_image, LOW_H_BLACK, LOW_S_BLACK, LOW_V_BLACK, HIGH_H_BLACK, HIGH_S_BLACK, HIGH_V_BLACK );

  //Combine both the images
  combined_image = combine_images(yellow_image, black_image);

  //Apply median blur to fill in the black holes
  medianBlur ( combined_image, combined_image, 15 );

  //Do Gaussian blur to smoothen out the image
  GaussianBlur( combined_image, combined_image, Size( 11, 11 ), 0, 0 );

  //Improve Contrast by equalizing the historgram
  equalizeHist(  combined_image, combined_image );
 
  //Find the largest contour in the image and send the response to the client
  largest_contour_data = find_contours(combined_image);

  std::string image_path = ros::package::getPath("terpcopter_vision") + "/results/boundary_detection.jpg";
  imwrite( image_path, largest_contour_data.image_of_contours );

  if(largest_contour_data.number_of_contours > 0 && largest_contour_data.area > CONTOUR_AREA_THRESHOLD)
  {
	response.detection_flag = true;
	response.center_x_pixel = largest_contour_data.center_x;
	response.center_y_pixel = largest_contour_data.center_y;
	response.area = largest_contour_data.area;
	response.arc_length = largest_contour_data.arc_length;

  }
 else
  {
 	response.detection_flag = false;
  }

  return true;
}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terpcopter_vision_server");
  ros::NodeHandle n;

  boundary_detection_service = n.advertiseService("detect_boundary", detect_boundary);
  ROS_INFO("Ready to detect boundary");

  ros::spin();

  return 0;
}	
