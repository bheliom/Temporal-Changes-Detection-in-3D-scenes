#include "pipelines.hpp"
#include "util/meshProcess.hpp"
#include "chngDet/chngDet.hpp"
#include "util/pbaDataInterface.h"
#include "common/globVariables.hpp"
#include "util/utilIO.hpp"


#include <iostream>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <time.h>

void energyMin(map<int, string> input_strings, double resolution){
  
  MeshChangeDetector mcd;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  getPlyFilePCL(input_strings[MESH], cloud);
  getPlyFilePCL(input_strings[CHANGEMASK], cloud2);

  mcd.energyMinimization(cloud, cloud2, resolution);
}

void pipelineCorrespondences(map<int,string> inputStrings, int K, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > new_cloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > view_points){

  vector<vector<vcg::Point3f> > tmp_3d_masks;
  vector<string> image_filenames, new_image_filenames;
  vector<CameraT> camera_data, newCameraData;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ searchPoint;
  
  vector<PtCamCorr> pt_cam_corr;
  vector<PtCamCorr> tmp_corr;
  map<int, vector<ImgFeature> > cam_feat_map;
  map<int, vector<ImgFeature> > cam_feat_map2;
  vector<vcg::Shot<float> > new_shots;

  //Make a backup copy of NVM file
  CmdIO::callCmd("cp "+inputStrings[BUNDLER]+" "+inputStrings[BUNDLER]+".bak");
  
  //Process NVM file to leave only one model
  std::ifstream inFile(inputStrings[BUNDLER].c_str());
  FileIO::forceNVMsingleModel(inFile, inputStrings[BUNDLER]);

  //Read NVM file
  FileIO::getNVM(inputStrings[BUNDLER], camera_data, image_filenames, pt_cam_corr, cam_feat_map);

  //Copy list of new images into NVM file directory(VisualSFM requirements)
  CmdIO::callCmd("cp "+inputStrings[PMVS]+" "+inputStrings[BUNDLER]+".txt");
  CmdIO vsfmHandler("./");

  //Run VisualSfM(currently to export matches it has to be done in two separate calls(VisualSfM bug))
  vsfmHandler.callVsfm(" sfm+resume+fixcam "+inputStrings[BUNDLER]+" "+inputStrings[OUTDIR]);
  vsfmHandler.callVsfm(" sfm+skipsfm+exportp "+inputStrings[OUTDIR]+" out_matches.txt ");

  //After running VisualSfM in new NVM file, new images indeces will start at the end
  int start_idx = camera_data.size();

  vector<CameraT> tmp_camera_data;
  vector<string> tmp_image_filenames;
  vector<PtCamCorr> tmp_pt_cam_corr;
  map<int, vector<ImgFeature> > tmp_cam_feat_map;
  map<string, int> img_idx_map;

  //Read new NVM file
  img_idx_map = FileIO::getNVM(inputStrings[OUTDIR], tmp_camera_data, tmp_image_filenames, tmp_pt_cam_corr, tmp_cam_feat_map);
  
  //Get new image files directories
  FileIO::readNewFiles(inputStrings[PMVS], new_image_filenames);

  string  tmpString = "newNVM.nvm";
  FileProcessing fileProc;  
  
  //Vector that for each new image contains vector of old image matched and sorted depending on number of matches
  std::vector<std::vector<std::string> > tmp_vec_vec;
  vector<vector<vector<pair<int,int> > > > feat_pairs;  
  //Get nearest neighbors from image matches
  FileIO::getNewImgNN(new_image_filenames, tmp_vec_vec, "out_matches.txt", K, feat_pairs);

  //Process the NVM file to leave only new cameras parameters
  fileProc.procNewNVMfile(inputStrings[OUTDIR], new_image_filenames, tmpString);
  FileIO::getNVM(tmpString, newCameraData, new_image_filenames, tmp_corr, cam_feat_map2);
  new_shots = FileIO::nvmCam2vcgShot(newCameraData, new_image_filenames);
 
  vector<ImgFeature> new_imgs_feat, old_imgs_feat;
  set<int> new_imgs_idx, old_imgs_idx;
 
  //Clouds for visualization in GUI
  new_cloud->points.resize(newCameraData.size());
  view_points->points.resize(newCameraData.size());

  //File for visualization in GUI
  ofstream myfile;
  myfile.open ("neighbor_cameras.txt");

  set<int> detected_feat_indeces;
  set<int> gt_change_indeces;

  for(int i = 0 ; i < newCameraData.size(); i++){

    //Saving camera positions
    new_cloud->points[i] = PclProcessing::vcg2pclPt(new_shots[i].Extrinsics.Tra());
    view_points->points[i] = PclProcessing::vcg2pclPt(new_shots[i].GetViewPoint());

    // Get features from new image
    int tmp_idx = i + start_idx-1;
    new_imgs_feat.insert(new_imgs_feat.end(),tmp_cam_feat_map[tmp_idx].begin(),tmp_cam_feat_map[tmp_idx].end());
    new_imgs_idx.insert(tmp_idx);
    
    //Get features of K neighbors from old image set
    for(int j = 0 ; j < K ; j++){
      myfile << tmp_vec_vec[i][j] <<"\n";
      int old_img_idx = img_idx_map[tmp_vec_vec[i][j]];
      old_imgs_feat.insert(old_imgs_feat.end(), tmp_cam_feat_map[old_img_idx].begin(), tmp_cam_feat_map[old_img_idx].end());
	
      old_imgs_idx.insert(old_img_idx);
    }    
  }
  myfile.close();
  
  //Run detection using feature grouping
  vector<vcg::Point3f> out_pts_vect;
  vector<vcg::Color4b> pts_colors;
  cout<<"Total features to investigate: "<<new_imgs_feat.size()+old_imgs_feat.size()<<endl;
  cout<<"Old features: " <<old_imgs_feat.size()<<" New features: "<<new_imgs_feat.size()<<endl;
  cout<<"size: "<<new_imgs_idx.size()<<endl;

  vector<int> corr_indeces = ImgChangeDetector::imgFeatDiff(new_imgs_feat, old_imgs_feat, tmp_pt_cam_corr, new_imgs_idx, old_imgs_idx);
 
  for(int i = 0 ; i < corr_indeces.size() ; i++){
    cv::Point3i c = tmp_pt_cam_corr[corr_indeces[i]].ptc;
 
    out_pts_vect.push_back(tmp_pt_cam_corr[corr_indeces[i]].pts_3d);
    pts_colors.push_back(vcg::Color4b(c.x, c.y, c.z, 0));
  }

  tmp_3d_masks.push_back(out_pts_vect);

  cout<<"Number of masks:"<<tmp_3d_masks.size()<<endl;
  MeshIO::saveChngMask3d(tmp_3d_masks, pts_colors, "change_mask.ply");
}

void pipelineImgDifference(map<int,string> inputStrings, int K, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > new_cloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > view_points, int proj_method, double resolutionVox){

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

  vsfmHandler.callVsfm(" sfm+resume+fixcam "+inputStrings[BUNDLER]+" "+inputStrings[OUTDIR]);
  vsfmHandler.callVsfm(" sfm+skipsfm+exportp "+inputStrings[OUTDIR]+" out_matches.txt ");

  FileIO::readNewFiles(inputStrings[PMVS], new_image_filenames);

  //////// Correspondence search//////////////////////////////////////
  int start_idx = camera_data.size();

  vector<CameraT> tmp_camera_data;
  vector<string> tmp_image_filenames;
  vector<PtCamCorr> tmp_pt_cam_corr;
  map<int, vector<ImgFeature> > tmp_cam_feat_map;
  map<string, int> img_idx_map;

  img_idx_map = FileIO::getNVM(inputStrings[OUTDIR], tmp_camera_data, tmp_image_filenames, tmp_pt_cam_corr, tmp_cam_feat_map);
  std::vector<std::vector<std::string> > tmp_vec_vec;
  vector<vector<vector<pair<int,int> > > > feat_pairs;  
  FileIO::getNewImgNN(new_image_filenames, tmp_vec_vec, "out_matches.txt", K, feat_pairs);
  //////////////////////////////////////////////////////////


  string  tmpString = "newNVM.nvm";
  FileProcessing fileProc;  

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

  vector<ImgFeature> new_imgs_feat, old_imgs_feat;
  set<int> new_imgs_idx;

  new_cloud->points.resize(newCameraData.size());
  view_points->points.resize(newCameraData.size());

  //File for visualization in GUI
  ofstream myfile;
  ofstream myfile2;
  myfile.open("neighbor_cameras.txt");
  myfile2.open("transformation.txt");

  set<int> detected_feat_indeces;
  set<int> gt_change_indeces;

  for(int i = 0 ; i < newShots.size(); i++){

    searchPoint = PclProcessing::vcg2pclPt(newShots[i].Extrinsics.Tra());
    view_points->points[i] = PclProcessing::vcg2pclPt(newShots[i].GetViewPoint());

    vector<int> pointIdxNKNSearch(K);
    vector<cv::Mat> nn_imgs;

    if(ImgIO::getKNNcamData(kdtree, searchPoint, image_filenames, nn_imgs, K, pointIdxNKNSearch)>0){
      
      new_cloud->points[i] = searchPoint;
      cv::Mat newImg(getImg(new_image_filenames[i]));

      for(int j = 0 ; j < K ; j++){
	////////
	myfile << tmp_vec_vec[i][j] <<"\n";
	cv::Mat oldImg(getImg(tmp_vec_vec[i][j]));
	////////
	
	if(oldImg.size() != newImg.size())
	  if(oldImg.rows==newImg.cols && oldImg.cols == newImg.rows){
	    cv::transpose(oldImg, oldImg);
	    cv::flip(oldImg, oldImg, 1);
	  }
	  else
	    continue;
		
	//cv::Mat oldImg(nn_imgs[j]);	  
      	cv::Mat finMask, H;

	if(ImgProcessing::getImgFundMat(newImg, oldImg, H)){

	  ImgChangeDetector::imgDiffThres(newImg, oldImg, H, finMask);
	  
	  cv::Mat testImg;
	  cv::Mat fin_mask2;
	  cv::Mat psaImg;

	  warpPerspective(newImg, psaImg, H, oldImg.size());
	  
	  //Save new img, old img and change mask
	  stringstream tmp_if;
	  tmp_if<<i<<j;	  
	  cv::imwrite(tmp_if.str()+"old.jpg", oldImg);
	  cv::imwrite(tmp_if.str()+"new.jpg", psaImg);

	  warpPerspective(finMask, fin_mask2, H, finMask.size());	  
	  
	  oldImg.copyTo(testImg, 255 - fin_mask2);

	  std::vector<cv::Point2f> mask_pts;
	  std::vector<cv::Point2f> mask_pts2;

	  ImgIO::getPtsFromMask(fin_mask2, mask_pts);
	  ImgIO::getPtsFromMask(finMask, mask_pts2);

	  myfile2<<tmp_if.str()+"mask.jpg\n";
	  cv::imwrite(tmp_if.str()+"mask.jpg", fin_mask2);
	  cv::imwrite(tmp_if.str()+"mask2.jpg", finMask);

	  cout<<"Change mask detected points: "<<mask_pts.size()<<endl;
	  //OVERLAY THE MASK
	  
	  /*
	    for(int g = 0 ; g < mask_pts.size(); g++){
	    cv::Point2f tmp_pt2 = mask_pts[g];
	    testImg.at<cv::Vec3b>(tmp_pt2.y, tmp_pt2.x)[0] = 255;
	    testImg.at<cv::Vec3b>(tmp_pt2.y, tmp_pt2.x)[1] = 0;
	    testImg.at<cv::Vec3b>(tmp_pt2.y, tmp_pt2.x)[2] = 0;
	    }
	  */

	  switch(proj_method){

	  case 0:
	    {//TRIANGULATION
	      std::cout<<"Projection by triangulation in progress... img: "<<i<<std::endl;
	      //////////////
	      cv::Mat mask_3d_pts(ImgIO::projChngMaskTo3D(finMask, newShots[i], shots[img_idx_map[tmp_vec_vec[i][j]]], H));
	      ////////////////

	      //  cv::Mat mask_3d_pts(ImgIO::projChngMaskTo3D(finMask, newShots[i], shots[pointIdxNKNSearch[0]], H));
	      std::vector<vcg::Point3f> tmp_vec_pts;
	      DataProcessing::cvt3Dmat2vcg(mask_3d_pts, tmp_vec_pts);		    
	      tmp_3d_masks.push_back(tmp_vec_pts);
	      break;
	    }
	  case 1: 	    	    
	    // RAY SHOOTING
	    tmp_3d_masks.push_back(ImgIO::projChngMask(inputStrings[MESH], finMask, newShots[i], resolutionVox));
	    break;
	    
	  case 2:
	    {// POINT CORRESPONDENCES
	    std::cout<<"Projection through point correspondences in progress... img: "<<i<<std::endl;
	    int old_img_idx = img_idx_map[tmp_vec_vec[i][j]];
	    tmp_3d_masks.push_back(ImgIO::projChngMaskCorr(fin_mask2, tmp_cam_feat_map[old_img_idx], pt_cam_corr, detected_feat_indeces));
	    tmp_3d_masks.push_back(ImgIO::projChngMaskCorr(finMask, tmp_cam_feat_map[start_idx+i-1], pt_cam_corr, detected_feat_indeces));
	    }
	    break;
	    
	  case 3:
	    {//Get GT
	    cv::Mat new_img_gt_mask(getImg("GT/"+tmp_if.str()+"new_gt.jpg"));
	    cv::Mat old_img_gt_mask(getImg("GT/"+tmp_if.str()+"old_gt.jpg"));

	    int old_img_idx = img_idx_map[tmp_vec_vec[i][j]];
	    
	    ImgIO::projChngMaskCorr(new_img_gt_mask, tmp_cam_feat_map[start_idx+1-1], pt_cam_corr, gt_change_indeces);
	    ImgIO::projChngMaskCorr(old_img_gt_mask, tmp_cam_feat_map[old_img_idx], pt_cam_corr, gt_change_indeces);
	    }
	    break;
	  }
	}		
      }
    }
  }

  cout<<"Total detected unique change points:"<<detected_feat_indeces.size()<<endl;
  myfile.close();
  myfile2.close();
  vector<vcg::Color4b> pts_colors(0);
  MeshIO::saveChngMask3d(tmp_3d_masks, pts_colors, "change_mask.ply");
}


void pipelinePSA(map<int,string> inputStrings, int K, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > new_cloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > view_points, int proj_method, double resolutionVox){

  vector<vector<vcg::Point3f> > tmp_3d_masks;
  vector<vcg::Shot<float> > shots, newShots;
  vector<string> image_filenames, new_image_filenames;
  vector<CameraT> camera_data, newCameraData;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ searchPoint;
  
  // Vector storring structure with of 3D and their corresponding vectors of camera indeces, point indeces and coordinates on images

  vector<PtCamCorr> pt_cam_corr;
  vector<PtCamCorr> tmp_corr;

  //This map stores ImgFeature vector per camera index. Features in vectors are ordered so that their indeces correspond to point indeces
  map<int, vector<ImgFeature> > cam_feat_map;
  map<int, vector<ImgFeature> > cam_feat_map2;

  FileIO::getNVM(inputStrings[BUNDLER], camera_data, image_filenames, pt_cam_corr, cam_feat_map);
  shots = FileIO::nvmCam2vcgShot(camera_data, image_filenames);

  CmdIO::callCmd("cp "+inputStrings[PMVS]+" "+inputStrings[BUNDLER]+".txt");
  CmdIO vsfmHandler("./");

  vsfmHandler.callVsfm(" sfm+resume+fixcam "+inputStrings[BUNDLER]+" "+inputStrings[OUTDIR]);
  vsfmHandler.callVsfm(" sfm+skipsfm+exportp "+inputStrings[OUTDIR]+" out_matches.txt ");

  FileIO::readNewFiles(inputStrings[PMVS], new_image_filenames);

  //////// Correspondence search//////////////////////////////////////
  int start_idx = camera_data.size();

  vector<CameraT> tmp_camera_data;
  vector<string> tmp_image_filenames;
  vector<PtCamCorr> tmp_pt_cam_corr;
  map<int, vector<ImgFeature> > tmp_cam_feat_map;
  map<string, int> img_idx_map;

  img_idx_map = FileIO::getNVM(inputStrings[OUTDIR], tmp_camera_data, tmp_image_filenames, tmp_pt_cam_corr, tmp_cam_feat_map);
  std::vector<std::vector<std::string> > tmp_vec_vec;

  // This vector contains for each new image vector of K neighbors with a vector of feature pairs indeces between new image and the neigbhor

  vector<vector<vector<pair<int,int> > > > feat_pairs;  

  FileIO::getNewImgNN(new_image_filenames, tmp_vec_vec, "out_matches.txt", K, feat_pairs);
  //////////////////////////////////////////////////////////


  string  tmpString = "newNVM.nvm";
  FileProcessing fileProc;  

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

  vector<ImgFeature> new_imgs_feat, old_imgs_feat;
  set<int> new_imgs_idx;

  new_cloud->points.resize(newCameraData.size());
  view_points->points.resize(newCameraData.size());

  //File for visualization in GUI
  ofstream myfile;
  myfile.open ("../matlabFiles/neighbor_cameras_psa.txt");

  set<int> detected_feat_indeces;
  set<int> gt_change_indeces;

  for(int i = 0 ; i < newShots.size(); i++){
    searchPoint = PclProcessing::vcg2pclPt(newShots[i].Extrinsics.Tra());
    view_points->points[i] = PclProcessing::vcg2pclPt(newShots[i].GetViewPoint());
    new_cloud->points[i] = searchPoint;

    cv::Mat newImg(getImg(new_image_filenames[i]));
    myfile<<i<<"\n";
    
    stringstream tmp_if;
    tmp_if<<i;

    for(int j = 0 ; j < K ; j++){


      cv::Mat oldImg(getImg(tmp_vec_vec[i][j]));   
      cv::Mat finMask, H;
      tmp_if<<j;
      int old_img_idx = img_idx_map[tmp_vec_vec[i][j]];
      
      vector<pair<int,int> > tmp_pair_vec = feat_pairs[i][j];
      vector<cv::Point2f> tmp_pts1(tmp_pair_vec.size());
      vector<cv::Point2f> tmp_pts2(tmp_pair_vec.size());

      for(int k = 0 ; k < tmp_pair_vec.size(); k++){
	cv::Point2f tmp_feat_coord;
	cv::Point2f tmp_feat_coord2;

	ImgFeature feat1 = tmp_cam_feat_map[i+start_idx-1][tmp_pair_vec[k].second];
	ImgFeature feat2 = tmp_cam_feat_map[old_img_idx][tmp_pair_vec[k].first];

	tmp_pts1[k] = cv::Point2f(feat1.y ,feat1.x);
	tmp_pts2[k] = cv::Point2f(feat2.y ,feat2.x);       
      }

      H = cv::findHomography(tmp_pts1, tmp_pts2, CV_RANSAC, 10);
        
      //      if(ImgProcessing::getImgFundMat(newImg, oldImg, H)){
      if(true){
	
	ImgChangeDetector::imgDiffThres(newImg, oldImg, H, finMask);

	cv::Mat testImg;
	cv::Mat psaImg;
	
	warpPerspective(newImg, psaImg, H, oldImg.size());

	cv::imwrite(tmp_if.str()+".jpg", psaImg);
	myfile<<tmp_if.str()+".jpg\n";

	cv::Mat fin_mask2;

	//This warp is to get points in second view
	warpPerspective(finMask, fin_mask2, H, finMask.size());	  
	oldImg.copyTo(testImg, 255 - fin_mask2);
	
	std::vector<cv::Point2f> mask_pts;
	ImgIO::getPtsFromMask(fin_mask2, mask_pts);
	
	switch(proj_method){
	  
	case 0:
	  {//TRIANGULATION
	    std::cout<<"Projection by triangulation in progress... img: "<<i<<std::endl;
	    //////////////
	    cv::Mat mask_3d_pts(ImgIO::projChngMaskTo3D(finMask, newShots[i], shots[img_idx_map[tmp_vec_vec[i][j]]], H));
	    ////////////////
	    
	    //  cv::Mat mask_3d_pts(ImgIO::projChngMaskTo3D(finMask, newShots[i], shots[pointIdxNKNSearch[0]], H));
	    std::vector<vcg::Point3f> tmp_vec_pts;
	    DataProcessing::cvt3Dmat2vcg(mask_3d_pts, tmp_vec_pts);		    
	    tmp_3d_masks.push_back(tmp_vec_pts);
	    break;
	  }
	case 1: 	    	    
	  // RAY SHOOTING
	  tmp_3d_masks.push_back(ImgIO::projChngMask(inputStrings[MESH], finMask, newShots[i], resolutionVox));
	    break;
	    
	case 2:
	  // POINT CORRESPONDENCES
	  std::cout<<"Projection through point correspondences in progress... img: "<<i<<std::endl;
	  int old_img_idx = img_idx_map[tmp_vec_vec[i][j]];
	  tmp_3d_masks.push_back(ImgIO::projChngMaskCorr(fin_mask2, cam_feat_map[old_img_idx], pt_cam_corr, detected_feat_indeces));
	  break;
	}	
      }
    }
  }
  
  vector<vcg::Color4b> pts_colors(0);
  MeshIO::saveChngMask3d(tmp_3d_masks, pts_colors, "change_mask.ply");
}

//////////////////////// UNFINISHED VISIBILITY ESTIMATION ////////////////////////////////////////////


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

