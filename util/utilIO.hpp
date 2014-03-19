#ifndef __UTILIO_H_INCLUDED__
#define __UTILIO_H_INCLUDED__

#include <string>
#include <unistd.h>
#include <map>

#include "../common/common.hpp"

#include<wrap/io_trimesh/import_off.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

void readCmdInput(std::map<int,std::string> &inputStrings, int argc, char** argv);

void inputHandler(std::vector<std::string> inputStrings);

void callVsfm(std::string dataPath, std::string outputPath);

void getPlyFileVcg(std::string filename, MyMesh &m);

void savePlyFileVcg(std::string filename, MyMesh &m);

void getBundlerFile(std::string filename);

void getBundlerFile(MyMesh &m, std::string filename, std::string filename_images, std::vector<vcg::Shot<float> > &shots, std::vector<std::string> &image_filenames);

template <typename T>
void visibilityEstimation(MyMesh &m, boost::shared_ptr<pcl::PointCloud<T> > pmvsCloud, int K){
  
  pcl::KdTreeFLANN<T> kdtree;
  kdtree.setInputCloud(pmvsCloud);
  pcl::PointXYZ searchPoint;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);


for(int i = 0; i < m.vert.size(); i++){
  searchPoint.x = m.vert[i].P().X();
  searchPoint.y = m.vert[i].P().Y();
  searchPoint.z = m.vert[i].P().Z();

  if(kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)){}

 }
}

template <typename T>
void getPlyFilePCL(std::string filename, boost::shared_ptr<pcl::PointCloud<T> > outCloud){

if(pcl::io::loadPLYFile<T> (filename, *outCloud) == -1)
  PCL_ERROR ("Couldn't read file\n"); 
}

#endif
