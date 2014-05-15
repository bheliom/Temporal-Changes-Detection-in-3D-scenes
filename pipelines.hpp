#ifndef __PIPELINES_H_
#define __PIPELINES_H_


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <string>
#include <map>

using namespace std;

void testEnerMin(map<int,string> input_strings);

void testBundler(string filename, string filename2, string filename3);
void test2();
void test3(string filename);
void testRemove(string filename);
void testNN(map<int,string> inputStrings);
void testVid(map<int,string> inputStrings);
void testNVM(map<int,string> inputStrings);
void testNewNVM(map<int,string> inputStrings);
void testPipeline(map<int,string> inputStrings);
void testProjections(map<int,string> inputStrings);
void pipelineCorrespondences(map<int,string> inputStrings, int K, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >);

#endif
