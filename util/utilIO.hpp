#ifndef __UTILIO_H_INCLUDED__
#define __UTILIO_H_INCLUDED__

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include "../common/common.hpp"
#include<wrap/io_trimesh/import_off.h>

void callVsfm(std::string dataPath, std::string outputPath);

template <typename T>
void getPlyFile(std::string filename, boost::shared_ptr<pcl::PointCloud<T> > outCloud){

if(pcl::io::loadPLYFile<T> (filename, *outCloud) == -1)
  PCL_ERROR ("Couldn't read file\n"); 
}

void getPlyFileVcg(std::string filename, MyMesh &m);


#endif
