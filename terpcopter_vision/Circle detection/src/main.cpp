/*Saimouli katragadda

Detects red circles

RGB-> HSV space-> detect red-> circles
*/


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

void brightness(Mat);

int main(int argc, char **argv) {

	VideoCapture cap(0);

	if (!cap.isOpened()) {
		cout << "Cannot open the webcam" << endl;
		return -1;
	}

	while (true) {
		Mat frame, hsv_frame;
		cap >> frame;
		cvtColor(frame, hsv_frame, COLOR_BGR2GRAY);
		medianBlur(frame, frame, 3);

		//BGR to HSV
		cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

		//keep only red pixels
		Mat lower_red_hue_range, higher_red_hue_range;

		inRange(hsv_frame, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_red_hue_range);
		inRange(hsv_frame, Scalar(160, 100, 100), Scalar(179, 255, 255), higher_red_hue_range);

		//combine two frames
		Mat red_hue_image;
		addWeighted(lower_red_hue_range, 1.0, higher_red_hue_range, 1.0, 0.0, red_hue_image);

		GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

		//hough transform to detect circles 
		vector<cv::Vec3f> circles;
		HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT,
			1, //accumulator resolution
			red_hue_image.rows / 4,
			200,  //minimum distance between two circles
			100, //canny high threshold
			0, 0); // 0 min radius, 1 max radius

		for (size_t current_circles = 0; current_circles < circles.size(); current_circles++) {
			Point center(round(circles[current_circles][0]), round(circles[current_circles][1]));
			int radius = round(circles[current_circles][2]);

			circle(frame, center, radius, Scalar(0, 255, 0), 3, 8, 0);
		}

		imshow("Combined threshold images", red_hue_image);
		imshow("detected frame", frame);

		if (waitKey(30) >= 0) break;

	}
	return 0;
	system("pause");

}

void brightness(Mat frame) {
	Mat imgH = frame + Scalar(50, 50, 50); //increases the brightness by 75 units
}


/*
//Code for changing Brightness and contrast
iBrightness = iSliderValue1 - 50;
dContrast = iSliderValue2 / 50.0;
frame.convertTo(src, -1, dContrast, iBrightness);
*/
