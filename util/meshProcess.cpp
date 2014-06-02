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
#include <pcl/kdtree/kdtree_flann.h>

#include <ctime>

typedef vcg::tri::UpdateTopology<MyMesh>::PEdge SingleEdge;

/**
   Function returns index of the first encountered voxel that is occluded. It is a modification of rayTraversal function implemented in PCL library class VoxelGridOcclusionEstimation.
*/
int rayBox::getFirstOccl(const Eigen::Vector4f& origin, const Eigen::Vector4f& direction, const float t_min){
  // coordinate of the boundary of the voxel grid
  Eigen::Vector4f start = origin + t_min * direction;
   
  // i,j,k coordinate of the voxel were the ray enters the voxel grid
  Eigen::Vector3i ijk = getGridCoordinatesRound (start[0], start[1], start[2]);
   
  // steps in which direction we have to travel in the voxel grid
  int step_x, step_y, step_z;
   
  // centroid coordinate of the entry voxel
  Eigen::Vector4f voxel_max = getCentroidCoordinate (ijk);
   
  if (direction[0] >= 0)
    {
      voxel_max[0] += leaf_size_[0] * 0.5f;
      step_x = 1;
    }
  else
    {
      voxel_max[0] -= leaf_size_[0] * 0.5f;
      step_x = -1;
    }
  if (direction[1] >= 0)
    {
      voxel_max[1] += leaf_size_[1] * 0.5f;
      step_y = 1;
    }
  else
    {
      voxel_max[1] -= leaf_size_[1] * 0.5f;
      step_y = -1;
    }
  if (direction[2] >= 0)
    {
      voxel_max[2] += leaf_size_[2] * 0.5f;
      step_z = 1;
    }
  else
    {
      voxel_max[2] -= leaf_size_[2] * 0.5f;
      step_z = -1;
    }
   
  float t_max_x = t_min + (voxel_max[0] - start[0]) / direction[0];
  float t_max_y = t_min + (voxel_max[1] - start[1]) / direction[1];
  float t_max_z = t_min + (voxel_max[2] - start[2]) / direction[2];
        
  float t_delta_x = leaf_size_[0] / static_cast<float> (fabs (direction[0]));
  float t_delta_y = leaf_size_[1] / static_cast<float> (fabs (direction[1]));
  float t_delta_z = leaf_size_[2] / static_cast<float> (fabs (direction[2]));
   
  // index of the point in the point cloud
  int index = -1;
   
  while ( (ijk[0] < max_b_[0]+1) && (ijk[0] >= min_b_[0]) && 
	  (ijk[1] < max_b_[1]+1) && (ijk[1] >= min_b_[1]) && 
	  (ijk[2] < max_b_[2]+1) && (ijk[2] >= min_b_[2]) )
    {
      index = this->getCentroidIndexAt (ijk);
      if (index != -1)
	return index;
   
      // estimate next voxel
      if(t_max_x <= t_max_y && t_max_x <= t_max_z)
	{
	  t_max_x += t_delta_x;
	  ijk[0] += step_x;
	}
      else if(t_max_y <= t_max_z && t_max_y <= t_max_x)
	{
	  t_max_y += t_delta_y;
	  ijk[1] += step_y;
	}
      else
	{
	  t_max_z += t_delta_z;
	  ijk[2] += step_z;
	}
    }
  return index;
}

/**
Function converts OpenCV matrix of 3D points into vector of VCG points
*/
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
/**
Function performs simple change detection operation for two images based on image difference and thresholding using Otsu algorithm
*/
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

  //-- Step 1: Detect the keypoints using Detector
  int minHessian = 400;

  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher > matcher;

  //  detector = new cv::OrbFeatureDetector(minHessian);
  //  extractor = new cv::OrbDescriptorExtractor;

  detector = new  cv::SurfFeatureDetector( minHessian );
  extractor = new  cv::SurfDescriptorExtractor;

  //    detector = new  cv::SiftFeatureDetector(minHessian);
  // extractor = new  cv::SiftDescriptorExtractor;

  matcher = new cv::FlannBasedMatcher;

  std::vector<cv::KeyPoint> keyPtsImg1, keyPtsImg2;

  detector->detect(img1, keyPtsImg1);
  detector->detect(img2, keyPtsImg2);

  //-- Step 2: Calculate descriptors (feature vectors)
  cv::Mat descriptors_object, descriptors_scene;

  extractor->compute(img1, keyPtsImg1, descriptors_object);
  extractor->compute(img2, keyPtsImg2, descriptors_scene);

  if ( descriptors_object.empty() || descriptors_scene.empty())
    return false;

  if(descriptors_object.type()!=CV_32F) {
    descriptors_object.convertTo(descriptors_object, CV_32F);
  }

  if(descriptors_scene.type()!=CV_32F) {
    descriptors_scene.convertTo(descriptors_scene, CV_32F);
  }

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  std::vector< cv::DMatch > matches;
  matcher->match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 1000;

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
 
  for( int i = 0; i < good_matches.size(); i++ )
    {
      //-- Get the keypoints from the good matches
      img1_pts.push_back( keyPtsImg1[ good_matches[i].queryIdx ].pt );
      img2_pts.push_back( keyPtsImg2[ good_matches[i].trainIdx ].pt );
    }

  if(img1_pts.size()<4 || img2_pts.size()<4)
    return false;

  H = cv::findHomography(img1_pts, img2_pts, CV_RANSAC, 10);

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
  outFile<<"0";
  outFile.close();

  std::cout<<"Done."<<std::endl;

}

/**
Function converts PCL 3D point into VCG point
*/
vcg::Point3f PclProcessing::pcl2vcgPt(pcl::PointXYZ inPt){
  vcg::Point3f outPt;
 
  outPt[0] = inPt.x;
  outPt[1] = inPt.y;
  outPt[2] = inPt.z;
 
  return outPt;
}

vcg::Point3f PclProcessing::pcl2vcgPt(pcl::PointXYZRGBA inPt){
  vcg::Point3f outPt;
 
  outPt[0] = inPt.x;
  outPt[1] = inPt.y;
  outPt[2] = inPt.z;
 
  return outPt;
}


/**
Function converts VCG 3D point into PCL point
*/
pcl::PointXYZ PclProcessing::vcg2pclPt(vcg::Point3<float> inPt){
  pcl::PointXYZ outPt;
  outPt.x = inPt.X();
  outPt.y = inPt.Y();
  outPt.z = inPt.Z();
  
  return outPt;
}

void PclProcessing::changeCloudColor(pcl::PointCloud<pcl::PointXYZRGBA> &in_cloud, int r, int g, int b){

  for(int i = 0 ; i < in_cloud.points.size(); i++){
    int32_t rgb = (r << 16) | (g << 8) | b; 
    in_cloud.points[i].rgb = *(float *)(&rgb); // makes the point red
  }
}

void PclProcessing::getROCparameters(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > gt_cloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > change_cloud, std::map<std::string,double> &parameters_map, const double &distance_threshold, const double &cloud_size){

  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;

  std::set<int> tp_set;
  double fp, fn, tn;

  int K = 1;

  kdtree.setInputCloud(gt_cloud);

  fp = 0;

  
  for(int i = 0; i < change_cloud->points.size(); i++){
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    
    pcl::PointXYZRGBA searchPoint = change_cloud->points[i];
    if(kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
      if(pointNKNSquaredDistance[0]<=distance_threshold)
	tp_set.insert(pointIdxNKNSearch[0]);
      else
	fp++;
    }
  }
  
  std::set<int>::iterator it;

  fn=0;
  for(int i = 0 ; i < gt_cloud->points.size(); i++){
    if(tp_set.find(i)==tp_set.end())
      fn++;
  }
  
  parameters_map["TP"] = tp_set.size();
  parameters_map["FP"] = fp;
  parameters_map["TN"] = cloud_size - change_cloud->points.size();
  parameters_map["FN"] = fn;
  
}


/**
Function calculates image coordinates of the point projected using vcg::Shot class member function
*/
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
