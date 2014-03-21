#ifndef MESHPROCESS_H
#define MESHPROCESS_H

#include "../common/common.hpp"
#include "utilIO.hpp"
#include <map>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

#include <cmath>

double getEdgeAverage(MyMesh &m);

double getFaceEdgeAverage(MyFace &f);

void removeUnnFaces(MyMesh &m, int thresVal);

void findOcc(std::map<int,int> inMap, std::vector<int> &outVector, int noOfOut);

template <typename T>
void visibilityEstimation(MyMesh &m, MyMesh &pmvsMesh, boost::shared_ptr<pcl::PointCloud<T> > pmvsCloud, int K, boost::shared_ptr<pcl::PointCloud<T> > mCloud, std::vector<vcg::Shot<float> > shots, std::vector<std::string> image_filenames){
  
  pcl::KdTreeFLANN<T> kdtree;
  pcl::KdTreeFLANN<T> kdtreeNeigh;

  pcl::PointXYZ searchPoint;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  std::map<int,int> tmpMap;
  std::vector<int> tmpSetImgs;

  std::vector<vcg::Point2f> tmpProjPoints;
  std::vector<cv::Mat> imageSet;
  
  int vertNumber;
  int tmpIdImg;
  int tmpCorrNum = 0;
  int tmpCount = 0;

  /*Instead of using 7 ring neighborhood we use 7 times the average length of an edge as a search radius for the neighbouring vertices*/
  float tmpRadius = 7*getEdgeAverage(m);

  MyMesh::PerVertexAttributeHandle<vcg::tri::io::CorrVec> named_hv = vcg::tri::Allocator<MyMesh>:: GetPerVertexAttribute<vcg::tri::io::CorrVec> (pmvsMesh ,std::string("correspondences"));
  
  kdtree.setInputCloud(pmvsCloud);
  kdtreeNeigh.setInputCloud(mCloud);

  vertNumber = m.VN();

  /*Iterate through each vertex in the input mesh*/
  std::cout<<"Start visibility estimation."<<std::endl;
  for(int i = 0; i < vertNumber; i++){
    
    if(i%1000==0) DrawProgressBar(40, (double)i/(double)vertNumber);

    searchPoint.x = m.vert[i].P().X();
    searchPoint.y = m.vert[i].P().Y();
    searchPoint.z = m.vert[i].P().Z();
    
    /*Find K nearest neighbors of given vertex*/
    if(kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)){
      
      tmpCount = pointIdxNKNSearch.size();

      /*Iterate through neighbors*/
      for (int j = 0; j < tmpCount; j++){
	tmpCorrNum = named_hv[pointIdxNKNSearch[j]].size();
	/*Iterate through corresponding images for given neighbour to find the one most often occuring one*/
	for(int k = 0; k < tmpCorrNum; k++){

	  tmpIdImg = named_hv[pointIdxNKNSearch[j]].at(k).id_img;

	  if(tmpMap.find(tmpIdImg)!=tmpMap.end())
	    tmpMap[tmpIdImg] += 1;
	  else
	    tmpMap[tmpIdImg] = 1;
	}
      }

      if(tmpMap.size()){	
	// Find 9 most often occuring ones
	findOcc(tmpMap, tmpSetImgs, 9);
	tmpMap.clear();
      }
    }

    if(tmpSetImgs.size()>0){
      /*here tmpSetImgs has 9 most often occuring images on which we have to project neighboring vertices of vertex V in 7 ring neighborhood*/
      if(kdtreeNeigh.radiusSearch(searchPoint, tmpRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance)){
	for(std::vector<int>::iterator it = tmpSetImgs.begin(); it!=tmpSetImgs.end(); ++it){       
	  cv::Mat image = getImg("/home/bheliom/Pictures/NotreDame/"+image_filenames[*it]);
	  //	  cv::Scalar color = cv::Scalar(255, 255, 255 );	
	  
	  for(int t = 0 ; t < pointIdxRadiusSearch.size(); t++){
	    
	    vcg::Point3f tmpPoint(mCloud->points[pointIdxRadiusSearch[t]].x, mCloud->points[pointIdxRadiusSearch[t]].y, mCloud->points[pointIdxRadiusSearch[t]].z);
	    
	    tmpProjPoints.push_back(shots[*it].Project(tmpPoint));
	    //	  std::cout<<"X:"<<tmpProjPoints[t].X()<<"Y:"<<tmpProjPoints[t].X()<<std::endl;
	    
	    //	  cv::circle(image, cv::Point(std::abs(tmpProjPoints[t].X()),std::abs(tmpProjPoints[t].Y())), 50 , color, 15);
	  }	
	  /*Here tmpProjPoints stores all projections of neighbor points on given image*/
	  
	  /*
	    cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.
	    cv::imshow( "Display window", image );                   // Show our image inside it.
	    
	    cv::waitKey(0);                        
	  */
	  tmpProjPoints.clear();
	}
      }
    }
    

    pointIdxNKNSearch.clear();
    pointIdxRadiusSearch.clear();
    tmpSetImgs.clear();

    /*
      TODO:
      -function finding nearest neighbors
      -function getting images
      -function projecting vertices on images
      -function to get intensity value
      -function to determine cloudy images
      
      -function to check occlusions
    */
  }
  DrawProgressBar(40, 1);
  std::cout<<"\n";
}
  
#endif
