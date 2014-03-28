#include <iostream>
#include "util/meshProcess.hpp"

using namespace std;

void testBundler(string filename, string filename2, string filename3);
void test2();
void test3(string filename);
void testRemove(string filename);
void testNN(map<int,string> inputStrings);

int main(int argc, char** argv){

  map<int,string> inputStrings;
  readCmdInput(inputStrings, argc, argv);

  testNN(inputStrings);
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

void testNN(map<int,string> inputStrings){

  pcl::PointCloud<pcl::PointXYZ>::Ptr pmvsCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud(new pcl::PointCloud<pcl::PointXYZ>);



  MyMesh m;
  MyMesh pmvsMesh;

  vector<vcg::Shot<float> > shots;
  vector<string> image_filenames;

  getPlyFileVcg(inputStrings[MESH], m);
  getPlyFileVcg(inputStrings[PMVS], pmvsMesh);

  //  getPlyFilePCL(inputStrings[PMVS], pmvsCloud);
  //  getPlyFilePCL(inputStrings[MESH], mCloud);
 
  getBundlerFile(pmvsMesh, inputStrings[BUNDLER], inputStrings[IMAGELIST], shots, image_filenames); 

  MyMesh::PerVertexAttributeHandle<vcg::tri::io::CorrVec> named_hv = vcg::tri::Allocator<MyMesh>:: GetPerVertexAttribute<vcg::tri::io::CorrVec> (pmvsMesh, std::string("correspondences"));


  for(int i = 0 ; i < pmvsMesh.vert.size(); i++)
    {

      if(!named_hv[i].empty())
	{
	  
	  for(int j = 0; j< named_hv[i].size(); j++)
	    {
	      int idImg = named_hv[i].at(j).id_img;
	      cv::Mat image = getImg(image_filenames[idImg]);
	      cv::Size s = image.size();

	      vcg::Point2i tmpDisp(s.width/2+named_hv[i].at(j).x,s.height/2-named_hv[i].at(j).y);
	      vcg::Point3f tmpDisp2 = pmvsMesh.vert[i].P();
	      
	      vcg::Point2i tmpDisp3 = getPtImgCoord(shots[idImg].Project(tmpDisp2), shots[i]);
	      vcg::Point2f tmpDisp4 = shots[idImg].Project(tmpDisp2);
	      tmpDisp4[0] = s.width-tmpDisp4.X();
	      tmpDisp4[1] = s.height - tmpDisp4.Y();


	      pmvsMesh.vert[i].SetS();
      
	      vcg::tri::UpdateSelection<MyMesh>::FaceFromVertexLoose(m);
	      vcg::tri::UpdateColor<MyMesh>::PerFaceConstant(pmvsMesh,vcg::Color4b::Red, true);
	      vcg::tri::UpdateColor<MyMesh>::PerVertexConstant(pmvsMesh,vcg::Color4b::Red, true);
      
	      savePlyFileVcg("testColor3.ply", pmvsMesh);

	      std::cout<<"Dimensions:"<< s.width << " " << s.height << std::endl;
	      std::cout<<"SIFT:"<< tmpDisp.X()<<" "<<tmpDisp.Y()<<std::endl;
	      std::cout<<"Point X Y:" << tmpDisp3.X()<< " " <<tmpDisp3.Y() << std::endl;
	      std::cout<<"difference"<< tmpDisp.X()-tmpDisp3.X() << " "<<tmpDisp.Y()-tmpDisp3.Y()<<std::endl;
	      static cv::Scalar color = cv::Scalar(255, 0, 0);	
	      static cv::Scalar color1 = cv::Scalar(0, 255, 0);	
	      cv::circle(image, cv::Point(tmpDisp4.X(),tmpDisp4.Y()), 50 , color1, 15);
	      cv::circle(image, cv::Point(tmpDisp.X(),tmpDisp.Y()), 50 , color, 15);
	      dispProjPt(tmpDisp3, image);
	      dispProjPt(tmpDisp, image);
	    }
	}
    }

  
  //  visibilityEstimation(m, pmvsMesh, pmvsCloud, 30, mCloud, shots, image_filenames);
}
