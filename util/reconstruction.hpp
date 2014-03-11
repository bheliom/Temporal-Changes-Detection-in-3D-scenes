#ifndef __RECONSTRUCTION_H_INCLUDED__
#define __RECONSTRUCTION_H_INCLUDED__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <vector>
#include <string>

/*
  Module responsible for reconstruction steps
*/

// Interface for VisualSfM //

/*
template <typename T>
void surfaceReconstruct( , ){

  pcl::Poisson<pcl::PointNormal> surfRecon;
  surfRecon.setInputCloud(cloud_with_normals); 
  surfRecon.setSearchMethod(tree2); 
  surfRecon.setConfidence(false); 
  surfRecon.setManifold(false); 
  surfRecon.setOutputPolygons(false); 
  surfRecon.setDepth(10); 
  
  surfRecon.setSolverDivide(4); 
  surfRecon.setIsoDivide(4); 
  surfRecon.setSamplesPerNode(1); 

}
*/

#endif
