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
void pipelineCorrespondences(map<int,string> inputStrings);

int main(int argc, char** argv){
  
  map<int,string> inputStrings;
  readCmdInput(inputStrings, argc, argv);

  //testPipeline(inputStrings);
  pipelineCorrespondences(inputStrings);

  return 0;
}


void pipelineCorrespondences(map<int,string> inputStrings){

  vector<vector<vcg::Point3f> > tmp_3d_masks;
  vector<vcg::Shot<float> > shots, newShots;
  vector<string> image_filenames, new_image_filenames;
  vector<CameraT> camera_data, newCameraData;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ searchPoint;
  
  vector<PtCamCorr> pt_cam_corr;
  vector<PtCamCorr> tmp_corr;
  map<int, vector<ImgFeature> > cam_feat_map;
  map<int, vector<ImgFeature> > cam_feat_map2;

  FileIO::getNVM(inputStrings[BUNDLER], camera_data, image_filenames, pt_cam_corr, cam_feat_map);
  shots = FileIO::nvmCam2vcgShot(camera_data, image_filenames);

  CmdIO::callCmd("cp "+inputStrings[PMVS]+" "+inputStrings[BUNDLER]+".txt");
  CmdIO vsfmHandler("./");

  //HOOK TO WORK WITHOUT VisualSFM
  vsfmHandler.callVsfm(" sfm+resume+fixcam "+inputStrings[BUNDLER]+" "+inputStrings[OUTDIR]);

  /////////////////////////////
  //testing new algorithm
  
  int start_idx = camera_data.size();

  vector<CameraT> tmp_camera_data;
  vector<string> tmp_image_filenames;
  vector<PtCamCorr> tmp_pt_cam_corr;
  map<int, vector<ImgFeature> > tmp_cam_feat_map;
  FileIO::getNVM(inputStrings[OUTDIR], tmp_camera_data, tmp_image_filenames, tmp_pt_cam_corr, tmp_cam_feat_map);
  
  /////////////////////////

  FileIO::readNewFiles(inputStrings[PMVS], new_image_filenames);
  string  tmpString = "newNVM.nvm";
  FileProcessing fileProc;  

  //HOOK TO WORK WITHOUT VisualSFM
  fileProc.procNewNVMfile(inputStrings[OUTDIR], new_image_filenames, tmpString);

  FileIO::getNVM(tmpString, newCameraData, new_image_filenames, tmp_corr, cam_feat_map2);
  newShots = FileIO::nvmCam2vcgShot(newCameraData, new_image_filenames);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
  cloud->points.resize(shots.size());

  for (size_t i = 0; i < cloud->points.size (); ++i){
    cloud->points[i] = PclProcessing::vcg2pclPt(shots[i].Extrinsics.Tra());
  }

  kdtree.setInputCloud(cloud);
  int K = 1;

  vector<ImgFeature> new_imgs_feat, old_imgs_feat;
  set<int> new_imgs_idx;

  for(int i = 0 ; i < newShots.size(); i++){
    searchPoint = PclProcessing::vcg2pclPt(newShots[i].Extrinsics.Tra());

    vector<int> pointIdxNKNSearch(K);
    vector<cv::Mat> nn_imgs;
    
    if(ImgIO::getKNNcamData(kdtree, searchPoint, image_filenames, nn_imgs, K, pointIdxNKNSearch)>0){
      /////////////////////////
      /// testing new algorithm
      
      int tmp_idx = i + start_idx-1;
      new_imgs_feat.insert(new_imgs_feat.end(),tmp_cam_feat_map[tmp_idx].begin(),tmp_cam_feat_map[tmp_idx].end());
      new_imgs_idx.insert(i+start_idx-1);
      
      //////////////////////////
      
      for(int j = 0 ; j < K ; j++){
	///////////////////////////
	/// testing new algorithm
	old_imgs_feat.insert(old_imgs_feat.end(), tmp_cam_feat_map[pointIdxNKNSearch[j]].begin(), tmp_cam_feat_map[pointIdxNKNSearch[j]].end());
	//////////////////////////
	
      }
    }
  }
  tmp_3d_masks.push_back(ImgChangeDetector::imgFeatDiff(new_imgs_feat, old_imgs_feat, tmp_pt_cam_corr, new_imgs_idx));
  MeshIO::saveChngMask3d(tmp_3d_masks, "chngMask_ImgAbsDiffK1.ply");
}

void testProjections(map<int,string> inputStrings){
  MyMesh m;
  getPlyFileVcg(inputStrings[MESH],m);
}

void testPipeline(map<int,string> inputStrings){

  vector<vector<vcg::Point3f> > tmp_3d_masks;
  vector<vcg::Shot<float> > shots, newShots;
  vector<string> image_filenames, new_image_filenames;
  vector<CameraT> camera_data, newCameraData;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ searchPoint;
  
  vector<PtCamCorr> pt_cam_corr;
  vector<PtCamCorr> tmp_corr;
  map<int, vector<ImgFeature> > cam_feat_map;
  map<int, vector<ImgFeature> > cam_feat_map2;

  FileIO::getNVM(inputStrings[BUNDLER], camera_data, image_filenames, pt_cam_corr, cam_feat_map);
  shots = FileIO::nvmCam2vcgShot(camera_data, image_filenames);

  CmdIO::callCmd("cp "+inputStrings[PMVS]+" "+inputStrings[BUNDLER]+".txt");
  CmdIO vsfmHandler("./");

  //HOOK TO WORK WITHOUT VisualSFM
  vsfmHandler.callVsfm(" sfm+resume+fixcam "+inputStrings[BUNDLER]+" "+inputStrings[OUTDIR]);

  FileIO::readNewFiles(inputStrings[PMVS], new_image_filenames);
  string  tmpString = "newNVM.nvm";
  FileProcessing fileProc;  

  //HOOK TO WORK WITHOUT VisualSFM
  fileProc.procNewNVMfile(inputStrings[OUTDIR], new_image_filenames, tmpString);

  FileIO::getNVM(tmpString, newCameraData, new_image_filenames, tmp_corr, cam_feat_map2);
  newShots = FileIO::nvmCam2vcgShot(newCameraData, new_image_filenames);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
  cloud->points.resize(shots.size());

  for (size_t i = 0; i < cloud->points.size (); ++i){
    cloud->points[i] = PclProcessing::vcg2pclPt(shots[i].Extrinsics.Tra());
  }

  kdtree.setInputCloud(cloud);
  int K = 1;

  vector<ImgFeature> new_imgs_feat, old_imgs_feat;
  set<int> new_imgs_idx;

  for(int i = 0 ; i < newShots.size(); i++){
    searchPoint = PclProcessing::vcg2pclPt(newShots[i].Extrinsics.Tra());

    vector<int> pointIdxNKNSearch(K);
    vector<cv::Mat> nn_imgs;
    
    if(ImgIO::getKNNcamData(kdtree, searchPoint, image_filenames, nn_imgs, K, pointIdxNKNSearch)>0){

      cv::Mat newImg( getImg(new_image_filenames[i]) );

      for(int j = 0 ; j < K ; j++){
	cv::Mat oldImg(nn_imgs[j]);	  
      	cv::Mat finMask, H;
	
	if(ImgProcessing::getImgFundMat(newImg, oldImg, H)){

	  ImgChangeDetector::imgDiffThres(newImg, oldImg, H, finMask);
	  cv::Mat testImg;
	  cv::Mat fin_mask2;

	  warpPerspective(finMask, fin_mask2, H, finMask.size());	  
	  oldImg.copyTo(testImg, 255 - fin_mask2);

	  std::vector<cv::Point2f> mask_pts;
	  ImgIO::getPtsFromMask(fin_mask2, mask_pts);
	  
	  cv::Size tmp_size =  fin_mask2.size();
	  
	  if(mask_pts.size()>((fin_mask2.rows*fin_mask2.cols)/4))
	    continue;
	  
	  //OVERLAY THE MASK
	  /*
	    for(int g = 0 ; g < mask_pts.size(); g++){
	    cv::Point2f tmp_pt2 = mask_pts[g];
	    testImg.at<cv::Vec3b>(tmp_pt2.y, tmp_pt2.x)[0] = 255;
	    testImg.at<cv::Vec3b>(tmp_pt2.y, tmp_pt2.x)[1] = 0;
	    testImg.at<cv::Vec3b>(tmp_pt2.y, tmp_pt2.x)[2] = 0;
	    }
	  */

	  /* TRIANGULATION	    
	     cv::Mat mask_3d_pts(ImgIO::projChngMaskTo3D(finMask, newShots[i], shots[pointIdxNKNSearch[0]], H));
	     std::vector<vcg::Point3f> tmp_vec_pts;
	     DataProcessing::cvt3Dmat2vcg(mask_3d_pts, tmp_vec_pts);		    
	     tmp_3d_masks.push_back(tmp_vec_pts);
	  */  
    
	  // RAY SHOOTING
	  // tmp_3d_masks.push_back(ImgIO::projChngMask(inputStrings[MESH], finMask, newShots[i]));
	
	  // POINT CORRESPONDENCES

	  tmp_3d_masks.push_back(ImgIO::projChngMaskCorr(fin_mask2, cam_feat_map[pointIdxNKNSearch[j]], pt_cam_corr));
	}		
      }
    }
  }
  MeshIO::saveChngMask3d(tmp_3d_masks, "chngMask_ImgAbsDiffK1.ply");
}

void testNewNVM(map<int,string> inputStrings){
  
  FileProcessing fileProc;
  
  ifstream inFile(inputStrings[IMAGELIST].c_str());
  
  string tmpString;
  vector<string> new_image_filenames;

  while(getline(inFile,tmpString))
    new_image_filenames.push_back(tmpString);
  
  fileProc.procNewNVMfile(inputStrings[MESH], new_image_filenames, "newNVM.nvm");
}

void testNVM(map<int,string> inputStrings){

  vector<CameraT> camera_data;
  vector<string> names;
  vector<PtCamCorr> tmp_corr;
  map<int, vector<ImgFeature> > cam_feat_map;

  FileIO::getNVM(inputStrings[MESH], camera_data, names, tmp_corr,cam_feat_map);

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
/*
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
*/
