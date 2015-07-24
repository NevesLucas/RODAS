///////////////////////////
/*
Collision detection and avoidance module header.
Lucas Neves and Emily Fitzgerald
6/23/2015
Rev 1.5
*/
//////////////////////////

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include "move_control.h"

using namespace std;
class collisionDetect
{
public:
	collisionDetect();

	void filter(double min, double max, string dim, pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
	
	void run(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, ros::Publisher cmd_vel_pub_);
	
private:
	int floor;
	int tooClose;
	float xConst1;
	float xConst2;
	float zConst1;
	float zConst2;
	moveControl move;
};

