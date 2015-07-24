///////////////////////////
/*
Turtelbot Motion Control Module.
Lucas Neves and Emily Fitzgerald
6/23/2015
Rev 1.0
*/
//////////////////////////
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <vector>

using namespace std;

class moveControl
{
	public:
		moveControl();

		void Apply(ros::Publisher cmd_vel_pub_);

	float angVel;  //desired velocities
	float linVel;

protected:
	int key;
	int lastPosX;
	int lastNegX;
	int lastPosZ;
	int lastNegZ;


	float currAngVel;
	float currLinVel;

	int imgCount;
};