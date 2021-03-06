#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#define PI 3.141592653589793

using namespace std;
void translatePC(float x, float y, float z, float angz, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_cloud)
{
	float theta = angz*PI; // TODO: convert angz to radians if needed

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation .
	transform_2.translation() << x, y, z;

	//  tetha radians arround Z axis
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	// apply transform_2
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

}