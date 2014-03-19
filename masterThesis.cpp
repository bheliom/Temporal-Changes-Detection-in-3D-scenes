#include <iostream>
#include "util/utilIO.hpp"
#include "util/meshProcess.hpp"

using namespace std;

void testBundler(string filename, string filename2, string filename3);
void test2();
void test3(string filename);
void testRemove(string filename);
void testNN(string filename);

int main(int argc, char** argv){

  string filename(argv[1]);
  //  string filename2(argv[2]);
  //  string filename3(argv[3]);

  // testBundler(filename, filename2, filename3);

  testNN(filename);

  return 0;
}

void test2(){

  string dataPath("./");
  string outputPath(".");

  callVsfm(dataPath, outputPath);
}

void test3(string filename){
  MyMesh m;
  getPlyFileVcg(filename, m);
}

void testRemove(string filename){

  MyMesh m;
  getPlyFileVcg(filename, m); 
  
  removeUnnFaces(m,20);

  savePlyFileVcg("testMesh.ply",m);
}


void testBundler(string filename, string filename2, string filename3){

  MyMesh m;
  getPlyFileVcg(filename, m); 

  vector<vcg::Shot<float> > shots;
  vector<string> image_filenames;

  getBundlerFile(m, filename2, filename3, shots, image_filenames);

  cout<<shots.size()<<endl;
  cout<<image_filenames.size()<<endl;
  cout<<image_filenames[0].c_str()<<endl;

  MyMesh::PerVertexAttributeHandle<vcg::tri::io::CorrVec> named_hv = vcg::tri::Allocator<MyMesh>:: GetPerVertexAttribute<vcg::tri::io::CorrVec> (m,std::string("correspondences"));

  cout<<named_hv[2].at(0).id_img<<endl;

}

void testNN(string filename){
  //  pcl::PointCloud<PointXYZ>::Pointer
  //  pcl::PointCloud<PointXYZ>::
  pcl::PointCloud<pcl::PointXYZ>::Ptr pmvsCloud;

  getPlyFilePCL(filename, pmvsCloud);
  
  
}
