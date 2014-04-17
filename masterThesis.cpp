#include <iostream>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>

#include "util/meshProcess.hpp"
#include "chngDet/chngDet.hpp"
#include "util/pbaDataInterface.h"
#include "common/globVariables.hpp"
#include "util/utilIO.hpp"

using namespace std;

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
int main(int argc, char** argv){
  
  map<int,string> inputStrings;
  readCmdInput(inputStrings, argc, argv);

  testPipeline(inputStrings);
  //  testNewNVM(inputStrings);
  //  testVid(inputStrings);
  return 0;
}

void testProjections(map<int,string> inputStrings){

  MyMesh m;
  getPlyFileVcg(inputStrings[MESH],m);

}

void testPipeline(map<int,string> inputStrings){

  //Get initial NVM file
  MyMesh m;
  getPlyFileVcg(inputStrings[MESH],m);

  vector<vcg::Shot<float> > shots;
  vector<vcg::Shot<float> > newShots;
  vector<string> image_filenames;
  vector<CameraT> camera_data;
  vector<CameraT> newCameraData;
  
  FileIO::getNVM(inputStrings[BUNDLER], camera_data, image_filenames);
  shots = FileIO::nvmCam2vcgShot(camera_data, image_filenames);
  
  CmdIO::callCmd("cp "+inputStrings[PMVS]+" "+inputStrings[BUNDLER]+".txt");

  //Call VisualSfM
  CmdIO vsfmHandler("./");
  vsfmHandler.callVsfm(" sfm+resume+fixcam "+inputStrings[BUNDLER]+" "+inputStrings[OUTDIR]);

  //Process the output file to get new cameras positions
  FileProcessing fileProc;

  //Read list of new files
  ifstream inFile(inputStrings[PMVS].c_str());
  string tmpString;
  vector<string> imgFilenames;

  while(getline(inFile,tmpString))
    imgFilenames.push_back(tmpString);

  inFile.close();

  tmpString = "newNVM.nvm";

  //Get positions of new cameras
  fileProc.procNewNVMfile(inputStrings[OUTDIR],imgFilenames, tmpString);

  FileIO::getNVM(tmpString, newCameraData, imgFilenames);
  newShots = FileIO::nvmCam2vcgShot(newCameraData, imgFilenames);

  //Find nearest neighbor
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
  cloud->points.resize (shots.size());

  for (size_t i = 0; i < cloud->points.size (); ++i){
    cloud->points[i].x = shots[i].Extrinsics.Tra().X();
    cloud->points[i].y = shots[i].Extrinsics.Tra().Y();
    cloud->points[i].z = shots[i].Extrinsics.Tra().Z();
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);
  pcl::PointXYZ searchPoint;

  for(int i = 0 ; i < newShots.size(); i++){

    searchPoint.x = newShots[i].Extrinsics.Tra().X();
    searchPoint.y = newShots[i].Extrinsics.Tra().Y();
    searchPoint.z = newShots[i].Extrinsics.Tra().Z();

    // K nearest neighbor search
    int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
   
    vector<cv::Mat> tmpImgs;

    if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){

      cv::Mat newImg = getImg(imgFilenames[i]);
      cv::Mat oldImg = getImg(image_filenames[pointIdxNKNSearch[0]]);
      cv::Mat outImgG = oldImg.clone();
      cv::Mat outImg;
      cv::Mat finMask;
      
      
      cv::Mat H = ImgProcessing::getImgFundMat(newImg, oldImg);

      warpPerspective(newImg, outImg, H, newImg.size());

      tmpImgs.push_back(outImg);

      cv::Mat diffImg = cv::abs(oldImg-outImg);

      warpPerspective(diffImg, outImgG, H, diffImg.size(), cv::WARP_INVERSE_MAP);
      
      cv::cvtColor(outImgG, finMask, CV_BGR2GRAY);      
      cv::threshold(finMask, finMask, 30, 1, CV_THRESH_OTSU);
      
      int no_of_elements = cv::sum(finMask).val[0]/255;
      int thres_val = (finMask.rows*finMask.cols)/3;

      cout<<"Thres:"<<thres_val<<" No of elem:"<<no_of_elements<<endl;

      if(no_of_elements < thres_val){
	std::cout<<"Jestem tutaj"<<std::endl;

	tmpImgs.push_back(finMask);      
	std::vector<cv::Point2f> cam1_points, cam2_points;
	std::cout<<"Jestem tutaj"<<std::endl;
	ImgIO::projChngMaskTo3D(finMask, newShots[i], shots[pointIdxNKNSearch[0]], H, cam1_points, cam2_points);
	std::cout<<"Jestem tutaj"<<std::endl;

	for(int k = 20 ; k < 40; k++){
	  cv::circle(newImg, cam1_points[k], 15, cv::Scalar(0,0,255),-1);
	  cv::circle(oldImg, cam2_points[k], 15, cv::Scalar(0,0,255),-1);
	}
	
	std::cout<<"teraz tutaj"<<std::endl;
	
	tmpImgs.push_back(newImg);
	tmpImgs.push_back(oldImg);
	
	std::cout<<"teraz tutaj"<<std::endl;
	ImgIO::dispImgs(tmpImgs); 
	tmpImgs.clear();
      }
    }
  }
}

void testNewNVM(map<int,string> inputStrings){
  
  FileProcessing fileProc;
  
  ifstream inFile(inputStrings[IMAGELIST].c_str());
  
  string tmpString;
  vector<string> imgFilenames;

  while(getline(inFile,tmpString))
    imgFilenames.push_back(tmpString);
  
  fileProc.procNewNVMfile(inputStrings[MESH], imgFilenames, "newNVM.nvm");
}

void testNVM(map<int,string> inputStrings){

  vector<CameraT> camera_data;
  vector<string> names;

  FileIO::getNVM(inputStrings[MESH], camera_data, names);

  vector<vcg::Shot<float> > shots = FileIO::nvmCam2vcgShot(camera_data, names);

}
void testVid(map<int,string> inputStrings){
  VidIO procVid(inputStrings[IMAGELIST]);
  procVid.saveImgFromVideo(inputStrings[OUTDIR], atoi(inputStrings[BUNDLER].c_str()));
}
void test2(){

  string dataPath("./");
  string outputPath(".");

  CmdIO testIt(dataPath);
  testIt.callVsfm(outputPath);
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

	      
	      for(int k = 1; k<100; k++)
		pmvsMesh.vert[i+k].SetS();
      
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
