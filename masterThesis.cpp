#include <iostream>
#include "util/utilIO.hpp"
#include "util/meshProcess.hpp"

using namespace std;
void test1(string filename);
void test2();
void test3(string filename);
void testRemove(string filename);

int main(int argc, char** argv){

  string filename(argv[1]);
  testRemove(filename);

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
}

void testRemove(string filename){
  MyMesh m;
  getPlyFileVcg(filename, m); 
 

  double avgEdge;
  avgEdge = getEdgeAverage(m);

  cout<<"Edge average length: "<<avgEdge<<endl;
  vcg::tri::UpdateTopology<MyMesh>::VertexFace(m); 
  
  MyMesh::FaceIterator fi;
  double tmp;
  int count = 0;

  for(fi=m.face.begin(); fi!=m.face.end(); ++fi){
   
    tmp = getFaceEdgeAverage(*fi);
    if(tmp>20*avgEdge) {
      vcg::tri::Allocator<MyMesh>::DeleteFace(m, *fi);
      count++;
    }
  }
  //  vcg::tri::UpdateColor<MyMesh>::PerFaceFromVertex(m);
  savePlyFileVcg("testMesh.ply",m);

  cout<<count<<endl;
}
