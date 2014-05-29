#ifndef __PIPELINES_H_
#define __PIPELINES_H_


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <string>
#include <map>

using namespace std;

void energyMin(map<int,string> input_strings, double);
void pipelineImgDifference(map<int,string> inputStrings, int K, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, int, double);
void pipelineCorrespondences(map<int,string> inputStrings, int K, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >);

void pipelinePSA(map<int,string> , int, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, int, double);

void generateGTcloud(map<int,string>, int , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > , int, double);
#endif
