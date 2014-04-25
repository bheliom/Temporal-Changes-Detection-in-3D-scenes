#include <fstream>
#include <map>
#include <boost/algorithm/string.hpp>
#include "meshProcess.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>

typedef vcg::tri::UpdateTopology<MyMesh>::PEdge SingleEdge;

void DataProcessing::cvt3Dmat2vcg(const cv::Mat &inMat, std::vector<vcg::Point3f> &out_pts){

  cv::Mat tmpMat;
  float w, x, y, z;

  for(int c = 0 ; c < inMat.cols; c++){
      
      tmpMat  = inMat.col(c);

      w = tmpMat.at<float>(3,0);
      x = tmpMat.at<float>(0,0)/w;
      y = tmpMat.at<float>(1,0)/w;
      z = tmpMat.at<float>(2,0)/w;
      
      out_pts.push_back(vcg::Point3f(x,y,z));
    }
}

cv::Mat ImgProcessing::diffThres(cv::Mat img1, cv::Mat img2){
  cv::Mat F;
  getImgFundMat(img1, img2, F);
  cv::Mat outImg;  
  cv::warpPerspective(img1, outImg, F, img2.size());
  
  return cv::abs(img2-outImg);
}

/**
Function finds fundamental matrix F for two input images. Function mostly based on OpenCV documentation tutorials.
 */
bool ImgProcessing::getImgFundMat(cv::Mat img1, cv::Mat img2, cv::Mat &H){

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  cv::SurfFeatureDetector detector( minHessian );
  std::vector<cv::KeyPoint> keyPtsImg1, keyPtsImg2;

  detector.detect( img1, keyPtsImg1 );
  detector.detect( img2, keyPtsImg2 );

  //-- Step 2: Calculate descriptors (feature vectors)
  cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors_object, descriptors_scene;

  extractor.compute( img1, keyPtsImg1, descriptors_object );
  extractor.compute( img2, keyPtsImg2, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  cv::FlannBasedMatcher matcher;
  std::vector< cv::DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 50;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

  std::vector< cv::DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
	{ good_matches.push_back( matches[i]); }
    }
 
  //-- Localize the object
  std::vector<cv::Point2f> img1_pts;
  std::vector<cv::Point2f> img2_pts;

  if(good_matches.size()>200)
    return false;
  
  std::cout<<"Good matches size:" << good_matches.size()<<std::endl;
  for( int i = 0; i < good_matches.size(); i++ )
    {
      //-- Get the keypoints from the good matches
      img1_pts.push_back( keyPtsImg1[ good_matches[i].queryIdx ].pt );
      img2_pts.push_back( keyPtsImg2[ good_matches[i].trainIdx ].pt );
    }

  H = cv::findHomography(img1_pts, img2_pts, CV_RANSAC, 5);

  return true;
}


/**
   Function splits given string depending on defined delimeter
*/
std::vector<std::string> FileProcessing::split(const std::string &inString, char delim){

  std::vector<std::string> outStrings;
  std::stringstream ss(inString);
  std::string item;

  while(std::getline(ss, item, delim))
    outStrings.push_back(item);

  return outStrings;
}

/**
   Function processes NVM file created after calling VisualSfM with new images and old model. It creates new NVM file consisting of camera positions for new images exclusively so that loadNVM function can be used.
*/
void FileProcessing::procNewNVMfile(const std::string &nvmFileDir, const std::vector<std::string> &imgFilenames, const std::string &outName){
  
  std::cout<<"Processing new NVM file..."<<std::endl;
  std::string tmpString;
  std::map<std::string, std::string> tmpMap;
  std::vector<std::string> tmpStrVec;

  std::ofstream outFile;
  outFile.open(outName.c_str());
  
  std::ifstream inFile(nvmFileDir.c_str());  

  //Create map with filenames
  for(int i = 0 ; i < imgFilenames.size() ; i++)
    tmpMap[imgFilenames[i]] = "";  
  
  //Get the header
  inFile>>tmpString;
  outFile<<tmpString+"\n\n";

  //Number of new cameras
  outFile<<imgFilenames.size()<<"\n\n";

  while(std::getline(inFile, tmpString))
    {      
      boost::split(tmpStrVec, tmpString, boost::is_any_of(" \t"));
      
      if(tmpMap.find(tmpStrVec[0])!=tmpMap.end())
	outFile << tmpString + "\n";      
      tmpStrVec.clear();
    }
  outFile.close();

  std::cout<<"Done."<<std::endl;

}

vcg::Point3f PclProcessing::pcl2vcgPt(pcl::PointXYZ inPt){
  vcg::Point3f outPt;
 
  outPt[0] = inPt.x;
  outPt[1] = inPt.y;
  outPt[2] = inPt.z;
 
  return outPt;
}


pcl::PointXYZ PclProcessing::vcg2pclPt(vcg::Point3<float> inPt){
  pcl::PointXYZ outPt;
  outPt.x = inPt.X();
  outPt.y = inPt.Y();
  outPt.z = inPt.Z();
  
  return outPt;
}

/**Function calculates image coordinates of the point projected using vcg::Shot class member function*/
vcg::Point2i getPtImgCoord(const vcg::Point2f &inPoint, const vcg::Shot<float> &inShot){
  
  vcg::Point2i tmpPoint(static_cast<int>(inPoint.X()), static_cast<int>(inPoint.Y()));

  return tmpPoint;
}


void findOcc(std::map<int, int> &inMap, std::vector<int> &outVector, int noOfOut){
  int tmpMax = 0;
  int tmpMaxId = 0;
  
  if(inMap.size() < noOfOut){
    for (std::map<int,int>::iterator it = inMap.begin(); it != inMap.end(); it++)
      outVector.push_back(it->first);
  }
  
  else {
    for (int k = 0; k < noOfOut; k++){
      for (std::map<int,int>::iterator it = inMap.begin(); it != inMap.end(); it++)
	if (tmpMax<it->second) {
	  tmpMax = it->second;
	  tmpMaxId = it->first;
	}
      outVector.push_back(tmpMaxId);
      inMap.erase(inMap.find(tmpMaxId));		
      tmpMax = 0;
      tmpMaxId = 0;
    }
  }
}

/**Function returns average length of the edge in the whole mesh*/
double getEdgeAverage(MyMesh &m){

  vcg::tri::UpdateTopology<MyMesh>::FaceFace(m);
  
  std::vector<SingleEdge> edges;
 
  vcg::tri::UpdateTopology<MyMesh>::FillUniqueEdgeVector(m,edges,true);

  MyMesh::VertexPointer vp1,vp2;
  
  vcg::Point3f tmp;
  double tmpSum = 0;
  double edgesSize = edges.size();

  std::cout<<"No of edges: "<<edgesSize<<std::endl;

  for(int i = 0; i<edgesSize; i++){

    vp1 = edges[i].v[0];
    vp2 = edges[i].v[1];
    
    tmp = vp1->P() - vp2->P();
    
    tmpSum+=sqrt(tmp.dot(tmp));
  }
  return tmpSum/edgesSize;
}

/**Function returns edge average length for given face*/
double getFaceEdgeAverage(MyFace &f){
  
  vcg::Point3f tmp1,tmp2,tmp3;
  
  tmp1 = f.cP(0) - f.cP(1);
  tmp2 = f.cP(0) - f.cP(2);
  tmp3 = f.cP(1) - f.cP(2);
  
  return (sqrt(tmp1.dot(tmp1))+sqrt(tmp2.dot(tmp2))+sqrt(tmp3.dot(tmp3)))/3;    
}

/**Function removes faces basing on the face average edge length*/
void removeUnnFaces(MyMesh &m, int thresVal){
  
  double avgEdge, tmp;
  MyMesh::FaceIterator fi;

  avgEdge = getEdgeAverage(m);

  std::cout<<"Edge average length: "<<avgEdge<<std::endl;
  
  vcg::tri::UpdateTopology<MyMesh>::VertexFace(m); 
  
  for(fi=m.face.begin(); fi!=m.face.end(); ++fi){
    tmp = getFaceEdgeAverage(*fi);

    if(tmp>thresVal*avgEdge)
      vcg::tri::Allocator<MyMesh>::DeleteFace(m, *fi);
  }  

}
