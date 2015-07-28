///////////////////////////
/*
Collision detection and avoidance module.
Lucas Neves and Emily Fitzgerald
6/23/2015
Rev 1.5
*/
//////////////////////////
#include "src\collisionDetect.h"

collisionDetect::collisionDetect()
{
	floor = 75;
	xConst1 = 0.0;
	xConst2 = 1.0;
	zConst1 = 0.0;
	zConst2 = 0.5;

}
void collisionDetect::filter(double min, double max, string dim, pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(input);
	pass.setFilterFieldName(dim);
	pass.setFilterLimits(min, max);
	pass.filter(*output);

}

void collisionDetect::run(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, ros::Publisher cmd_vel_pub_)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	filter(zConst1, zConst2, "z", cloud, cloud_p);

	if (cloud_p->size() <= floor)
	{
		cout << "something's in front of us!" << endl; //movement goes here
		move.angVel = -1.7;
		move.linVel = 0;
		move.Apply(cmd_vel_pub_);
	}

	else
	{
		filter(-1.3, 0.14, "y", cloud_p, cloud_filtered);

		for (int i = 0; i < cloud_filtered->size(); i++)
		{
			double zcoord = cloud_filtered->points[i].z;
			if (zcoord <= 0.5)
			{
				tooClose++;
			}
		}
		if (tooClose >= 30)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr right(new pcl::PointCloud<pcl::PointXYZ>);

			move.linVel = 0;

			filter(0.0, 1.0, "x", cloud_filtered, right);

			pcl::PointCloud<pcl::PointXYZ>::Ptr left(new pcl::PointCloud<pcl::PointXYZ>);
			filter(-1.0, 0.0, "x", cloud_filtered, left);

			if (right->size() > left->size())
			{
				cout << "turn left \n"; //movement goes here
				move.angVel = 0.70;
				move.Apply(cmd_vel_pub_);
			}
			else if (right->size() < left->size())
			{
				cout << "turn  right \n"; //movement goes here
				move.angVel = -0.70;
				move.Apply(cmd_vel_pub_);
			}
			else
			{
				move.angVel = 0;
				move.linVel = 0.13;
				move.Apply(cmd_vel_pub_);
			}



		}

		else
		{
			move.angVel = 0;
			move.linVel = 0.15;
			move.Apply(cmd_vel_pub_);
		}

	}

}