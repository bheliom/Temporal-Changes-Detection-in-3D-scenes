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

  vcg::Color4b ver_col(1,2,3,0);

  // MyMesh m;
  
  // getPlyFileVcg(inputStrings[MESH], m);
  // ver_col[0] = m.vert[0].r;
  // ver_col[1] = m.vert[0].g;
  // ver_col[2] = m.vert[0].b;
  // std::cout << ver_col[0]<<" "<<ver_col[1]<<" "<<ver_col[2]<<std::endl;



  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  //pipelineImgDifference(inputStrings, 1 , cloud, 1);
  
  //   pipelineCorrespondences(inputStrings, 1, cloud, cloud2l);

  //inputStrings[CHANGEMASK] = "change_mask.ply";
  // testEnerMin(inputStrings);
  
  return 0;

}
