#include "common/common.hpp"
#include "pipelines.hpp"
#include "util/utilIO.hpp"

#include <map>
#include <string>

int main(int argc, char** argv){
  
  map<int,string> inputStrings;
  readCmdInput(inputStrings, argc, argv);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //testPipeline(inputStrings);
  //  pipelineCorrespondences(inputStrings, 1, cloud);
    testEnerMin(inputStrings);

  return 0;
}
