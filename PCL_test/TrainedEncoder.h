#ifndef _TRAINEDENCODER
#define _TRAINEDENCODER

//#include <cuda_runtime.h>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <string>
//#include <caffe/caffe.hpp>
//#include "caffe.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace caffe;

class TrainedEncoder{

public:

	//input: locations of network file, prototxt file, and label names file
	TrainedEncoder(string, string, const char*);

	~TrainedEncoder();

	//set GPU mode
	void useGPU();
	//set CPU mode
	void useCPU();

	//-----------------------------------------------

	//run a forward pass on this input image 
	int forwardImg(Mat&);

	//----------------------------------------------
	//get top-level features
	vector<float> getTopFeatures(string blobName);

	//return the top-K output classes (as pairs in the second argument)
	int getTopK(int K, vector<pair<float, int> >&);

	//gets the image classification
	int getTopClass(string&);

	//returns the class name from index
	string getClassName(int);

protected:

	string netBinaryFile;   //location of network to load
	string netProtoFile;    //location of proto file, which describes the net
	const char* labelNameFile;   //location of label file, which names the output units

	bool usingGPU;

	int deviceID;         //ID of the GPU (default 0)

	Net<float> caffeNet;  //Caffe-trained net

	vector<Mat> caffeImages;  //input image(s)  
	vector<int> caffeLabels;  //not used, but a needed input by caffe

	//network's data layer 
	//const shared_ptr<ImageDataLayer<float> > caffeImageLayer;

	//network classification results
	vector<Blob<float>*> myResults;

	vector<string> labelNames;  //vector of label names

	float loss;

	int topClass;

	void readLabelFile();

};

#endif
