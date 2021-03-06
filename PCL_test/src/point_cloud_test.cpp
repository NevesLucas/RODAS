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
//additional classes/files

////////////////////////CAFFE SETUP////////////////////////////////////////////
string direc ="/home/neuromorphicslab/catkin_ws/src/point_cloud_test/include/Trained_Networks/";
string netFile = direc + "caffenet_train_iter_3500.caffemodel";
string protoTxt = direc + "train_val.prototxt";
const char* labelFile = "/home/neuromorphicslab/catkin_ws/src/point_cloud_test/include/Trained_Networks/emily_labels.txt";
vector<float> featureResponses;
int topLabelID = 0;
string topLabelName="junk";
//network create
TrainedEncoder tNet(netFile,protoTxt,labelFile);
vector<pair<float, int> > topK; //topK responses and indices

////further setup
bool stoppedSpin = false;
bool seenBall = false;
bool seenCone = false;
bool seenChair = false;
ofstream myfile;
int lastSeen = 0;

//----------------------------------------------------------


ros::Publisher cmd_vel_pub_; //movement publisher

static bool stop =false;
//////////////
using namespace std;
using namespace cv;
int stepNum = 5;
float currHead = 0; 
float currPos[3], currOri[2];
static bool targetmsg =false;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

///////boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("New Viewer"));
static pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB);

collisionDetect avoidance;

PointCloud::Ptr PCmap;

/// need this //// void callback(const PointCloud::ConstPtr & msg)
void callback (const PointCloud::ConstPtr & msg)
{
///////////////////////////////////////////Avoid Collision Using Point Clouds//////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud(*msg,*input);

  avoidance.run(input, cmd_vel_pub_);
  //---------------------------------------------------------------------------------------------------------------------

/////////////////////////////////////////Attempted Mapping/////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reoriented(new pcl::PointCloud<pcl::PointXYZRGB>);

  	translate(currPos[1], currPos[2], currPos[0], -currOri[0], input,reoriented);
	cout << currPos[0] <<"  "<< currPos[1] << "  " <<currPos[2] << "  " << 0.8124*(currOri[0]*180)-4.7564<<endl;
 
  if(targetmsg == true)
  {
  
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
  	 pcl::copyPointCloud(*PCmap,*target);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
  	
  	mapping (reoriented,target,PCmap);
/*
  	pcl::ApproximateVoxelGrid<PointT> grid;
  	
  	
  	{
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (temp);
    grid.filter (*PCmap);
    stepNum = 2;
	}
	*/
	

	viewer.showCloud(PCmap);	

  }
  else
  {
  	PCmap =reoriented; 
  	//wait for next cloud
  	targetmsg=true;
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

	currHead = 2*asin(currOri[0]/sqrt(currOri[0]*currOri[0]+currOri[1]*currOri[1])); //speed
}
//-----------------------------------------------------------------------------------------

//////////////////////////////////Caffe Cont///////////////////////////////////////////
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		//grab image
		cv::Mat grabbedImg = cv_bridge::toCvShare(msg,"bgr8")->image;

		//classify
		tNet.forwardImg(grabbedImg);
		classifyAndPlotCaffe(grabbedImg,tNet,2,topK,myfile,"barLocal.dat");
		topLabelID = tNet.getTopClass(topLabelName);

		string class0 ("ball");
		string class1 ("cone");
		string class2 ("chair");

		if(topLabelName.compare(class0) == 0 && lastSeen != 0)
		{
				seenBall = true;
				lastSeen = 0;
		}

		if(topLabelName.compare(class1) == 0 && lastSeen != 1)
		{
				seenCone = true;
				lastSeen = 1;
		}

		if(topLabelName.compare(class2) == 0 && lastSeen != 2)
		{
				seenChair = true;
				lastSeen = 2;
		}
	}
	catch(cv_bridge::Exception& e)
	{
		//Ros_Error("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv) 
{

  ros::init(argc, argv, "point_cloud_test");
  ros::NodeHandle nh;
  
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); //advertise topic to publish to TB
 

  ros::Subscriber sub = nh.subscribe<PointCloud>("/voxel_grid/output",1,callback);
  ros::Subscriber odom_sub = nh.subscribe("/odom",1,OdomCallback);
  ros::Subscriber img_sub = nh.subscribe("camera/rgb/image_color",1,ImageCallback);

  
  ros::Rate loop_rate(60); //60 HZ
  while(ros::ok())
  {
	 loop_rate.sleep();
	 ros::spinOnce();
  }
}















/////LEGACY CODE/////////////
//TB stuff
/*
int key;
int lastPosX = 0;
int lastNegX = 0;
int lastPosZ = 0;
int lastNegZ = 0;


float angVel;  //desired velocities
float linVel;

float currAngVel;
float currLinVel;

int imgCount = 0;
ros::Publisher cmd_vel_pub_;


void imageCallbackMatt(const sensor_msgs::ImageConstPtr& msg)
{


  cv_bridge::CvImagePtr cv_ptr;

  //std::cout << "\nAGGGGG!";

  try {

cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    
    /*Mat blur_img;
    double minVal, maxVal;
    minMaxLoc(cv_ptr->image, &minVal, &maxVal, NULL, NULL);
    //cout << "\nMin and max " << minVal << " " << maxVal;
    cv_ptr->image.convertTo(blur_img, CV_8UC1, 255.0/(maxVal-minVal), -minVal * 255.0/(maxVal - minVal));

    imshow("view", blur_img);

    CObservation3DRangeScanPtr obsRGBD = blur_img;
    pcl::PointCloud&lt;pcl::PointXYZRGB&gt; cloud;
    obsRGBD-&gt;project3DPointsFromDepthImageInto(
      cloud,
      false
      );
    
////////////////MOVEMENT BELOW/////////////////
    


    
    //imshow("view", cv_ptr->image);
    //imshow("view", grabbedImg);

    // std::cout<< "\nYou should see something";

    //image_pub_.publish(cv_ptr->toImageMsg());

    //controller from keyboard
    //Wait only 1ms to provide smoother continuous movement
    key = waitKey(3);
    
    //Initialize direction to zero
    geometry_msgs:: Twist base_cmd;
    //base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;



    



    //Forward (i)
    if (key == 105) {
        if (lastPosX == 0) {
        linVel = 0.16;
        lastPosX = 0.16;
  }
        linVel = 0.16;
    }

    //Reverse (k)
    if (key == 107) {
        linVel = -0.15;
    }

    //Rotate left (j)
    if (key == 106) {
       angVel = 1.00;
    }

    //Rotate right (l)
    if (key == 108) {
       angVel = -1.00;
    }

    //Forward and turn left (u)
    if (key == 117) {
       linVel = 0.15;
       angVel = 0.75;
    }

    //Forward and turn right (o)
    if (key == 111) {
       linVel = 0.15;
       angVel = -0.75;
    }

    //Reverse and turn left (n)
    if (key == 110) {
      linVel = -0.15;
      angVel = 0.75;
    }

    //Reverse and turn right (.)
    if (key == 46) {
      linVel = -0.15;
      angVel = -0.75;
    }


    // if (lastPosX != 0 && key != 105) {
    //  base_cmd.linear.x = lastPosX - 0.04;
    //  }
    //Publish the movement commands
    
    currAngVel = 0.5 * currAngVel + 0.5 * angVel;
    currLinVel = 0.5 * currLinVel + 0.5 * linVel;

    base_cmd.angular.z = currAngVel; 
    base_cmd.linear.x = currLinVel;

    
    if (base_cmd.linear.x != 0 || base_cmd.angular.z != 0) {
       cmd_vel_pub_.publish(base_cmd);
    }

    //Kill program when ESC key pressed
    if (key == 27) {
        ros::shutdown();
  }

    //If 's' is pressed, save the image
    if (key == 115) {
        std::string filename;
        std::string imagename;
        std::string fileextension;
        imagename = "image";
        fileextension = ".jpg";
        std::stringstream sstm;
        sstm << imagename << imgCount << fileextension;
        filename = sstm.str();
        imwrite(filename, cv_bridge::toCvShare(msg, "bgr8")->image);
        imgCount++;
  }
  }

  catch (cv_bridge::Exception& e) {
    std::cout << "Could not convert (probably) from type" <<  msg->encoding.c_str() << " " <<  e.what();
  }

}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor (1.0, 0.5,1.0);
  pcl::PointXYZRGB o;
  o.x = 1.0;
  o.y = 0;
  o.z = 0;
  viewer.addSphere (o,.25,"sphere",0);
  std::cout<<"I only run once "<<std::endl;
}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
  static unsigned count = 0;
  std::stringstream ss;
  ss<<"Once perviewer loop: "<<count++;
  viewer.removeShape("text",0);
  viewer.addText(ss.str(),200,300,"text",0);
  user_data++;
}


*/
