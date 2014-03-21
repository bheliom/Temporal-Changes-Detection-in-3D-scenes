#ifndef MESHPROCESS_H
#define MESHPROCESS_H

#include "../common/common.hpp"
#include "utilIO.hpp"
#include <map>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

double getEdgeAverage(MyMesh &m);

double getFaceEdgeAverage(MyFace &f);

void removeUnnFaces(MyMesh &m, int thresVal);

void findOcc(std::map<int,int> inMap, std::vector<int> &outVector, int noOfOut);

template <typename T>
void visibilityEstimation(MyMesh &m, MyMesh &pmvsMesh, boost::shared_ptr<pcl::PointCloud<T> > pmvsCloud, int K){
  
  pcl::KdTreeFLANN<T> kdtree;
  pcl::PointXYZ searchPoint;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::map<int,int> tmpMap;
  std::vector<int> tmpSetImgs;

  int vertNumber;
  int tmpIdImg;
  int tmpCorrNum = 0;
  int tmpCount = 0;

  MyMesh::PerVertexAttributeHandle<vcg::tri::io::CorrVec> named_hv = vcg::tri::Allocator<MyMesh>:: GetPerVertexAttribute<vcg::tri::io::CorrVec> (pmvsMesh ,std::string("correspondences"));
  
  kdtree.setInputCloud(pmvsCloud);
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
    
    /*here tmpSetImgs has 9 most often occuring images on which we have to project neighboring vertices of vertex V in 7 ring neighborhood
      
      TODO:
      -function finding 7-ring neighborhood
      -function getting images
      -function projecting vertices on images
      -function to get intensity value
      -function to determine cloudy images

      -function to check occlusions
     */
  }
}
  
#endif
