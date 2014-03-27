#ifndef MESHPROCESS_H
#define MESHPROCESS_H

#include <map>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <cmath>
#include <vector>
#include <string>

#include "../common/common.hpp"
#include "utilIO.hpp"

vcg::Point2i getPtImgCoord(const vcg::Point2f &inPoint, const vcg::Shot<float> &inShot);

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
  int tmpCurrKneighId = 0;
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



    if(!named_hv[i].empty()){
      m.vert[i].SetS();
      
      cv::Mat image = getImg(image_filenames[named_hv[i].at(0).id_img]);

      vcg::Shot<float> szocik = shots[named_hv[i].at(0).id_img];

      vcg::tri::UpdateSelection<MyMesh>::FaceFromVertexLoose(m);
	
      vcg::tri::UpdateColor<MyMesh>::PerFaceConstant(m,vcg::Color4b::Red, true);

      vcg::tri::UpdateColor<MyMesh>::PerVertexConstant(m,vcg::Color4b::Red, true);
      
      savePlyFileVcg("testColor3.ply", m);
            
      vcg::Point2f tmpDisp2 = szocik.Intrinsics.Project(m.vert[i].P());

      vcg::Point2i tmpDisp = getPtImgCoord(tmpDisp2, szocik);
      
      std::cout<< tmpDisp2.X()<<" "<<tmpDisp2.Y()<<std::endl;
      std::cout<< tmpDisp.X()<<" "<<tmpDisp.Y()<<std::endl;
      std::cout<< szocik.Intrinsics.ViewportPx.X() <<" "<< szocik.Intrinsics.ViewportPx.Y() << std::endl;
      std::cout<< szocik.Intrinsics.CenterPx.X()<<" "<< szocik.Intrinsics.CenterPx.Y()<<std::endl;

      dispProjPt(tmpDisp, image);
    }

    searchPoint.x = m.vert[i].P().X();
    searchPoint.y = m.vert[i].P().Y();
    searchPoint.z = m.vert[i].P().Z();
    
    /*Find K nearest neighbors of given vertex*/
    if(kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)){
      
      tmpCount = pointIdxNKNSearch.size();

      /*Iterate through neighbors*/
      for (int j = 0; j < tmpCount; j++){
	tmpCurrKneighId = pointIdxNKNSearch[j];
	tmpCorrNum = named_hv[tmpCurrKneighId].size();
	/*Iterate through corresponding images for given neighbour to find the one most often occuring one*/
	for(int k = 0; k < tmpCorrNum; k++){
	  tmpIdImg = named_hv[tmpCurrKneighId].at(k).id_img;

	  if(tmpMap.find(tmpIdImg)!=tmpMap.end())
	    tmpMap[tmpIdImg] += 1;
	  else
	    tmpMap[tmpIdImg] = 1;
	}
      }

      if(!tmpMap.empty()){	
	// Find 9 most often occuring ones
	findOcc(tmpMap, tmpSetImgs, 9);
	tmpMap.clear();
      }
    }

    if(!tmpSetImgs.empty()){
      /*here tmpSetImgs has 9 most often occuring images on which we have to project neighboring vertices of vertex V in 7 ring neighborhood*/
      if(kdtreeNeigh.radiusSearch(searchPoint, tmpRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance)){
	for(std::vector<int>::iterator it = tmpSetImgs.begin(); it!=tmpSetImgs.end(); ++it){       
	  
	  cv::Mat image = getImg(image_filenames[*it]);

	  for(int t = 0 ; t < pointIdxRadiusSearch.size(); t++){
	    
	    vcg::Point3f tmpPoint(mCloud->points[pointIdxRadiusSearch[t]].x, mCloud->points[pointIdxRadiusSearch[t]].y, mCloud->points[pointIdxRadiusSearch[t]].z);	    
	    tmpProjPoints.push_back(shots[*it].Project(tmpPoint));
	    
	  }	


	  /*Here tmpProjPoints stores all projections of neighbor points on given image*/
	  
	  /*
	    for(int t = 0 ; t < tmpProjPoints.size(); t++){
	    vcg::Point2i tmpDisp = getPtImgCoord(tmpProjPoints[t], shots[*it]);
	
	    std::cout<<tmpDisp.X()<<" "<<tmpDisp.Y()<<std::endl;
	    dispProjPt(tmpDisp, image);
	    dispProjPt(shots[*it].Intrinsics.ViewportPx,image);
	    }

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
