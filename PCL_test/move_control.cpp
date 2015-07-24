#include "src\move_control.h"

moveControl::moveControl()
{
	lastPosX = 0;
	lastNegX = 0;
	lastPosZ = 0;
	lastNegZ = 0;
	imgCount = 0;


}

void moveControl::Apply(ros::Publisher cmd_vel_pub_)
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