/*!
Change detection main test function.
*/

#include "common/common.hpp"
#include "pipelines.hpp"
#include "util/utilIO.hpp"

#include <map>
#include <string>

/**Main function*/
int main(int argc, char** argv){
  
  map<int,string> inputStrings;
  readCmdInput(inputStrings, argc, argv);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  CmdIO::callCmd("cp "+inputStrings[BUNDLER]+" "+inputStrings[BUNDLER]+".bak");
  
  std::ifstream inFile(inputStrings[BUNDLER].c_str());
  FileIO::forceNVMsingleModel(inFile, inputStrings[BUNDLER]);


  //testPipeline(inputStrings);
  
  //  pipelineCorrespondences(inputStrings, 1, cloud);

  //inputStrings[CHANGEMASK] = "change_mask.ply";
  // testEnerMin(inputStrings);
  
  return 0;

}
