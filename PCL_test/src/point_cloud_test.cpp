///////////////////////////
/*
PROTOTYPE TURTLEBOT MASTER CONTROL
Lucas Neves and Emily Fitzgerald
6/23/2015
Rev 4.0
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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "collisionDetect.h"
#include "PCTranslate.h"
#include "PairAlignReg.h"

#include "TrainedEncoder.h"
#include "Plotting.h"

using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

////////////////////////CAFFE SETUP////////////////////////////////////////////
// set up some file paths
string direc = "/home/neuromorphicslab/catkin_ws/src/point_cloud_test/include/Trained_Networks/";
string netFile = direc + "caffenet_train_iter_3500.caffemodel";
string protoTxt = direc + "train_val.prototxt";
const char* labelFile = "/home/neuromorphicslab/catkin_ws/src/point_cloud_test/include/Trained_Networks/emily_labels.txt";

//some storage and label variables

vector<float> featureResponses;
int topLabelID = 0;
string topLabelName = "junk";

//network creater
TrainedEncoder tNet(string netFile, string protoTxt, const char* labelFile);
vector<pair<float, int> > topK;

////other setup
bool stoppedSpin = false;
bool seenBall = false;
bool seenCone = false;
bool seenChair = false;
ofstream myfile;
int lastSeen = 0;

//----------------------------------------------------------

ros::Publisher cmd_vel_pub_;

static bool stop = false;

//////////////
//these need to be removed:
int stepNum = 5;
float currHead = 0;
float currPos[3], currOri[2];
///

static bool targetmsg = false;

///////boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("New Viewer"));
static pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //this sets up the cloudviewer
//static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB);

collisionDetect avoidance; //initialize avoidance class

PointCloud::Ptr PCmap; //deprecated, will be removed

/// need this //// void callback(const PointCloud::ConstPtr & msg)
void callback(const PointCloud::ConstPtr & msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::copyPointCloud(*msg, *input);

	avoidance.run(input, cmd_vel_pub_);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reoriented(new pcl::PointCloud<pcl::PointXYZRGB>);

	translate(currPos[1], currPos[2], currPos[0], -currOri[0], input, reoriented);
	cout << currPos[0] << "  " << currPos[1] << "  " << currPos[2] << "  " << 0.8124*(currOri[0] * 180) - 4.7564 << endl;

	if (targetmsg == true)
	{

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*PCmap, *target);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

		mapping(reoriented, target, PCmap);

		pcl::ApproximateVoxelGrid<PointT> grid;


		{
			grid.setLeafSize(0.05, 0.05, 0.05);
			grid.setInputCloud(temp);
			grid.filter(*PCmap);
			stepNum = 2;
		}



		viewer.showCloud(PCmap);
	}
	else
	{
		PCmap = reoriented;
		//wait for next cloud
		targetmsg = true;
	}
}


/// get position info using odometry
void OdomCallback(const nav_msgs::Odometry::ConstPtr &OdomMsg)
{
	currPos[0] = OdomMsg->pose.pose.position.x;
	currPos[1] = OdomMsg->pose.pose.position.y;
	currPos[2] = OdomMsg->pose.pose.position.z;
	currOri[0] = OdomMsg->pose.pose.orientation.z;
	currOri[1] = OdomMsg->pose.pose.orientation.w;

	currHead = 2 * asin(currOri[0] / sqrt(currOri[0] * currOri[0] + currOri[1] * currOri[1])); //speed
}

//////////////////////////////////Caffe Cont///////////////////////////////////////////

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{  //cleanup this callback

	//grab image
	cv::Mat grabbedImg = cv_bridge::toCvShare(msg, "bgr8")->image;

	//classify
	tNet.forwardImg(grabbedImg);
	classifyAndPlotCaffe(grabbedImg, tNet, 2, topK, myfile, "barLocal.dat");
	topLabelID = tNet.getTopClass(topLabelName);

	string class0("ball");
	string class1("cone");
	string class2("chair");

	if (topLabelNAme.compare(class0) == 0)
	{
		if (stoppedSpin == true)
		{
			seenBall = false;
			seenCone = false;
			seenChair = false;
		}
		else if (lastSeen != 0)
		{
			seenBall = true;
			lastSeen = 0;
		}
	}

	if (topLabelName.compare(class1) == 0)
	{
		if (stoppedSpin == true)
		{
			seenBall = false;
			seenCone = false;
			seenChair = false;
		}
		else if (lastSeen != 1)
		{
			seenCone = true;
			lastSeen = 1;
		}
	}

	if (topLabelName.compare(class2) == 0)
	{
		if (stoppedSpin == true)
		{
			seenBall = false;
			seenCone = false;
			seenChair = false;
		}
		else if (lastSeen != 2)
		{
			seenChair = true;
			lastSeen = 2;
		}
	}

	///TODO: figure out the stoppedSpin stuff -------------------------------


}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "point_cloud_test");
	ros::NodeHandle nh;

	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); //advertise topic to publish to TB


	//ros::Subscriber sub = nh.subscribe<PointCloud>("/voxel_grid/output",1,callback);
	ros::Subscriber odom_sub = nh.subscribe("/odom", 1, OdomCallback);
	//ros::Subscriber img_sub = nh.subscribe("camera/rgb/image_color",1,ImageCallback);


	ros::Rate loop_rate(60); //60 HZ
	while (ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}
}


