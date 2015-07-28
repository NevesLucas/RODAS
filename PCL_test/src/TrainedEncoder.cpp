#include "TrainedEncoder.h" 
//-----------------------------
//Constructor
TrainedEncoder::TrainedEncoder(string _netBinaryFile, string _netProtoFile, const char*  _labelNameFile): caffeNet(_netProtoFile)
{
  netBinaryFile = _netBinaryFile;   
  netProtoFile = _netProtoFile;
  labelNameFile = _labelNameFile;
  
  Caffe::set_phase(Caffe::TEST);
  Caffe::set_mode(Caffe::GPU); //default: using GPU
  
  deviceID = 0;               //default device_id (GPU)
  Caffe::SetDevice(deviceID);
  
  //load network description from file
//  caffeNet.InitFromFile(netProtoFile);
  
  //load network binary
  caffeNet.CopyTrainedLayersFrom(netBinaryFile);

  //load label names
  readLabelFile();
 
}

//-----------------------------
//Destructor
TrainedEncoder::~TrainedEncoder(){
  
}

//------------------------------------
//toggle GPU
void TrainedEncoder::useGPU(){
  Caffe::set_mode(Caffe::GPU); 
  usingGPU = true;
}
void TrainedEncoder::useCPU(){
  Caffe::set_mode(Caffe::CPU); 
  usingGPU = false;
}

//---------------------------------------
//forwardImg
int TrainedEncoder::forwardImg(Mat& inImg){
   
  caffeImages.push_back(inImg);
  caffeLabels.push_back(0);
  
  //TODO: the name "data" is also dependant on prototxt
  const caffe::shared_ptr<ImageDataLayer<float> > caffeImageLayer =
     boost::static_pointer_cast<ImageDataLayer<float> >(
     caffeNet.layer_by_name("data"));
      
     
  caffeImageLayer->AddImagesAndLabels(caffeImages,caffeLabels);
  
  vector<Blob<float>* > dummy_bottom_vec;
  const vector<Blob<float>*>& result = caffeNet.Forward(dummy_bottom_vec, &loss);

  //store results
  myResults = result;
  
  caffeImages.pop_back();  //remove image
  caffeLabels.pop_back();  //remove label
}

//---------------------------------------------------------
//getTopFeatures
vector<float> TrainedEncoder::getTopFeatures(string blobName){
 
  //TODO: memory leak?
  vector<float> featureResponses;
  
  //blobName like "fc7"
  const caffe::shared_ptr<Blob<float> > featureBlob = caffeNet.blob_by_name(blobName);
	   
  int numFeatures = featureBlob->num();
  int dimFeatures = featureBlob->count() / numFeatures;
    
  float* featureBlobData;
  featureBlobData = featureBlob->mutable_cpu_data();
    
  for (int d = 0; d < dimFeatures; ++d) {
   //    std::cout << "\n" << feature_blob_data[d];
      featureResponses.push_back(featureBlobData[d]);
  }
      
  return featureResponses;
}
   
//sorting magic
struct sort_pred {
    bool operator()(const std::pair<float,int> &left, const std::pair<float,int> &right) {
        return left.first < right.first;
   }
};

//---------------------------------------------------------------------
//getTopK
//returns top output classes and their confidence (after softmax)
int TrainedEncoder::getTopK(int K, vector<pair<float, int> > &topK)
{
    topK.clear();
    topK.reserve(K);

    vector<pair<float,int> > featureIndex;

    vector<float> featureResponses;
    featureResponses = getTopFeatures("prob");

    featureIndex.reserve(featureResponses.size());

    for (int i=0;i<=featureResponses.size()-1; i++)
    {
        featureIndex.push_back(make_pair (featureResponses[i], i));

    }
    sort(featureIndex.begin(), featureIndex.end(), sort_pred());

    //for (int i=0; i<featureResponses.size()-1; i++)
    //{
    //    cout << "\nArrr matey: " << featureIndex[i].first << " " << featureIndex[i].second;
    //}

    int last = featureResponses.size()-1;
    for (int j=last; j>=last-K;j--)
    {
        topK.push_back(featureIndex[j]);
    }

    featureResponses.clear();
    featureIndex.clear();

    return 1;
}

//---------------------------------------------
//getClassName
string TrainedEncoder::getClassName(int index)
{
    return labelNames[index];
}

//----------------------------------------------------------------------
//getTopClass
int TrainedEncoder::getTopClass(string &label){
  
  const float* argmax = myResults[1]->cpu_data();   //[1] since proto has two output layers!  This depends on the prototxt!

  topClass = argmax[0];
  
  //convert to string~label
  label = labelNames[topClass];
  
  return topClass;
}
      
//-------------------------------------------------------------------------
//readLabelFile
void TrainedEncoder::readLabelFile(){
  
   fstream in(labelNameFile);
   
   string line;
   string comma = ",";

   while (getline(in, line)) 
   {
      istringstream lineStream(line);
  
      //parse line_stream into a vector of words
      vector<string> words(istream_iterator<string>(lineStream),
                          (istream_iterator<string>()));
      
      string myLabel;
      

      for (int j=0; j<words.size()-1; j++){

          //Limited to 5 words!  Otherwise it bleeds off the bottom edge of my plot!
          if (j < 5)
          {
              myLabel = myLabel + words[j+1];

              //if there's a comma, we're done (requested by Max)
             if (words[j+1].find(comma) != std::string::npos)
             {
                 break;
             }
          }
      }

      labelNames.push_back(myLabel);
    }
  
}


  
