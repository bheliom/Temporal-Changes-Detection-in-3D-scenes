#ifndef _CHNGDET_H_
#define _CHNGDET_H_

#include<vcg/space/point3.h>
#include "../common/common.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_adjacency_container.h>
#include <pcl/octree/octree_impl.h>

#include <set>
class ChangeDetector{

protected:
  MyMesh m;

public:

ChangeDetector(){};
void setInMesh(MyMesh inMesh)
  {vcg::tri::Append<MyMesh,MyMesh>::MeshCopy(m,inMesh);}

  virtual std::vector<float> getPofChange()=0;
  virtual std::vector<vcg::Point3f> getChangeMap()=0;

};

class ImgChangeDetector : public ChangeDetector{

protected:
  std::vector<std::string> newImgFilenames;

public:
  void setNewImgFilenames(std::vector<std::string> filenames)
  {newImgFilenames = filenames;}
  cv::Mat getImageDifference(cv::Mat, cv::Mat);
  std::vector<vcg::Point3f> projChngMask(cv::Mat, vcg::Shot<float>);
  static void imgDiffThres(cv::Mat, cv::Mat, cv::Mat, cv::Mat&);
  static std::vector<int> imgFeatDiff(const std::vector<ImgFeature>&, const std::vector<ImgFeature>&, const std::vector<PtCamCorr>&, const std::set<int>&, const std::set<int>&);
  static std::vector<int> filtColor(const std::vector<int>&, const std::vector<PtCamCorr>&, const std::vector<std::string>&);
};

class MeshChangeDetector : public ChangeDetector{

public:

MeshChangeDetector() : ChangeDetector(){};
  std::vector<float> getPofChange(){
  }
  std::vector<vcg::Point3f> getChangeMap(){
  }
  
  static void energyMinimization(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, double, const double&);

static int getRedCount(const std::vector<int>&, const pcl::PointCloud<pcl::PointXYZRGBA>&);
  
};

typedef pcl::octree::OctreeContainerPointIndices LeafContainerT;
typedef pcl::PointXYZRGBA PointT;
class MyOctree : public pcl::octree::OctreePointCloudSearch<PointT>{

public:

MyOctree (const double resolution) : pcl::octree::OctreePointCloudSearch<PointT> (resolution)
{
}

virtual
~MyOctree ()
{
}

LeafContainerT* findLeafN(unsigned int x, unsigned int y, unsigned z){
return findLeaf(x,y,z);
}
  
  void findNeighbors(const pcl::octree::OctreeKey&, std::vector<LeafContainerT*>&);
  void findNeighbors(const pcl::octree::OctreeKey&, std::vector<pcl::octree::OctreeKey>&);
};
#endif
