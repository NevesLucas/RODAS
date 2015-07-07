#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());


/*  METHOD #2: Using a Affine3f
This method is easier and less error prone
*/
Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

// Define a translation of 2.5 meters on the x axis.
transform_2.translation() << 2.5, 0.0, 0.0;

// The same rotation matrix as before; tetha radians arround Z axis
transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

// Print the transformation
printf("\nMethod #2: using an Affine3f\n");
std::cout << transform_2.matrix() << std::endl;

// Executing the transformation
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
// You can either apply transform_1 or transform_2; they are the same
pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2