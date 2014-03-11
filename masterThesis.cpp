#include <iostream>
#include "util/utilIO.hpp"
#include "util/meshProcess.hpp"

using namespace std;
void test1(string filename);
void test2();
void test3(string filename);

int main(int argc, char** argv){

  string filename(argv[1]);
  test3(filename);

  return 0;
}

void test1(string filename){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  getPlyFile<pcl::PointXYZRGB>(filename, cloud);
  
}

void test2(){

  string dataPath("./");
  string outputPath(".");

  callVsfm(dataPath, outputPath);
}

void test3(string filename){
  MyMesh m;
  getPlyFileVcg(filename, m);
  cout<<"Edge average length: "<<getEdgeAverage(m)<<endl;
}
