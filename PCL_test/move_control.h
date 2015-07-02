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
	moveControl(){
		lastPosX = 0;
		lastNegX = 0;
		lastPosZ = 0;
		lastNegZ = 0;
		imgCount = 0;

		
	}

	void Apply(ros::Publisher cmd_vel_pub_)
	{
	

		//Initialize direction to zero
		geometry_msgs::Twist base_cmd;

		currAngVel = 0.5 * currAngVel + 0.5 * angVel;
		currLinVel = 0.5 * currLinVel + 0.5 * linVel;

		base_cmd.angular.z = currAngVel;
		base_cmd.linear.x = currLinVel;

		if (base_cmd.linear.x != 0 || base_cmd.angular.z != 0) {

			cmd_vel_pub_.publish(base_cmd);
		}

	}

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