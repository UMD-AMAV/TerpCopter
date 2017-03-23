//
// THIS FILE CONTAINS THE ARDUINO INTERFACE FOR EXTERNAL COMPASS AND SERVO
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

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Wire.h>
#include <HMC5883L.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Range.h>

///////////////////////////////////////////
//
//	VARIABLES
//
///////////////////////////////////////////

unsigned long timestamp;
const int time_threshold = 50;
char frameid[] = "/external_compass_link";
Servo servo;
HMC5883L compass;
ros::NodeHandle  nh;
sensor_msgs::Range compass_msg;
float z_heading ;
float z_degrees ;
ros::Publisher pub_yaw( "terpcopter/external_compass", &compass_msg);

///////////////////////////////////////////
//
//	ROS SUBSCRIBER
//
///////////////////////////////////////////

void servo_cmd( const sensor_msgs::Range& cmd_msg)
{
  servo.write(cmd_msg.range); //set servo angle, should be from 0-180  

}

ros::Subscriber<sensor_msgs::Range> sub("terpcopter/servo", servo_cmd);


///////////////////////////////////////////
//
//	SETUP
//
///////////////////////////////////////////

void setup()
{
 nh.initNode(); 
 nh.advertise(pub_yaw);
 nh.subscribe(sub);
 Wire.begin();
 compass = HMC5883L();
 compass.SetScale(1.3);   
 compass.SetMeasurementMode(Measurement_Continuous);
 compass_msg.header.frame_id =  frameid;  

 servo.attach(6);  // Attaches the servo on pin 6 to the servo object
}

///////////////////////////////////////////
//
//	LOOP
//
///////////////////////////////////////////

void loop()
{ 
         MagnetometerRaw raw = compass.ReadRawAxis();
         MagnetometerScaled scaled = compass.ReadScaledAxis();
         
	 z_heading = atan2(scaled.ZAxis, scaled.YAxis);
         if(z_heading < 0) z_heading += 2*PI;
         if(z_heading > 2*PI) z_heading -= 2*PI;
         
	 z_degrees = z_heading * 180/M_PI;


         if ( (millis()-timestamp) > time_threshold)
         {
	      	compass_msg.range = z_degrees;              
         	compass_msg.header.stamp = nh.now();
         	pub_yaw.publish(&compass_msg);
		timestamp = millis();
	}

         nh.spinOnce();
         delay(1);	//Sleep for 1 milli second
}
