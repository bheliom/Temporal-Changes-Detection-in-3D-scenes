#include "utilIO.hpp"
#include "pbaDataInterface.h"
#include "meshProcess.hpp"
#include "../common/globVariables.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

#include <fstream>
#include <sstream>
#include <cstdlib>
#include "opencv2/calib3d/calib3d.hpp"
#include <ctime>

/**
   Function returns index of the first encountered voxel that is occluded. It is a modification of rayTraversal function implemented in PCL library class VoxelGridOcclusionEstimation.
*/

int rayBox::getFirstOccl(const Eigen::Vector4f& origin, const Eigen::Vector4f& direction, const float t_min){
  // coordinate of the boundary of the voxel grid
  Eigen::Vector4f start = origin + t_min * direction;
   
  start*=0.3;
  // i,j,k coordinate of the voxel were the ray enters the voxel grid
  Eigen::Vector3i ijk = getGridCoordinatesRound (start[0], start[1], start[2]);
   
  // steps in which direction we have to travel in the voxel grid
  int step_x, step_y, step_z;
   
  // centroid coordinate of the entry voxel
  Eigen::Vector4f voxel_max = getCentroidCoordinate (ijk);
   
  if (direction[0] >= 0)
    {
      voxel_max[0] += leaf_size_[0] * 0.5f;
      step_x = 1;
    }
  else
    {
      voxel_max[0] -= leaf_size_[0] * 0.5f;
      step_x = -1;
    }
  if (direction[1] >= 0)
    {
      voxel_max[1] += leaf_size_[1] * 0.5f;
      step_y = 1;
    }
  else
    {
      voxel_max[1] -= leaf_size_[1] * 0.5f;
      step_y = -1;
    }
  if (direction[2] >= 0)
    {
      voxel_max[2] += leaf_size_[2] * 0.5f;
      step_z = 1;
    }
  else
    {
      voxel_max[2] -= leaf_size_[2] * 0.5f;
      step_z = -1;
    }
   
  float t_max_x = t_min + (voxel_max[0] - start[0]) / direction[0];
  float t_max_y = t_min + (voxel_max[1] - start[1]) / direction[1];
  float t_max_z = t_min + (voxel_max[2] - start[2]) / direction[2];
        
  float t_delta_x = leaf_size_[0] / static_cast<float> (fabs (direction[0]));
  float t_delta_y = leaf_size_[1] / static_cast<float> (fabs (direction[1]));
  float t_delta_z = leaf_size_[2] / static_cast<float> (fabs (direction[2]));
   
  // index of the point in the point cloud
  int index = -1;
   
  while ( (ijk[0] < max_b_[0]+1) && (ijk[0] >= min_b_[0]) && 
	  (ijk[1] < max_b_[1]+1) && (ijk[1] >= min_b_[1]) && 
	  (ijk[2] < max_b_[2]+1) && (ijk[2] >= min_b_[2]) )
    {
      index = this->getCentroidIndexAt (ijk);
      if (index != -1)
	return index;
   
      // estimate next voxel
      if(t_max_x <= t_max_y && t_max_x <= t_max_z)
	{
	  t_max_x += t_delta_x;
	  ijk[0] += step_x;
	}
      else if(t_max_y <= t_max_z && t_max_y <= t_max_x)
	{
	  t_max_y += t_delta_y;
	  ijk[1] += step_y;
	}
      else
	{
	  t_max_z += t_delta_z;
	  ijk[2] += step_z;
	}
    }
  return index;
}

/**
   Function creates a mesh from 3D change mask points and saves the PLY file.
*/
void MeshIO::saveChngMask3d(const std::vector<std::vector<vcg::Point3f> > &pts_3d, const std::string &name){
  
  std::cout<<"Saving change 3D mask.."<<std::endl;
  
  MyMesh m;
  float x, y, z;
  int count = 0;

  for(int i = 0 ; i < pts_3d.size() ; i++){
    DrawProgressBar(40, (double)i/(double)pts_3d.size());

    for(int j = 0 ; j < pts_3d[i].size(); j++){
      x = pts_3d[i].at(j).X();
      y = pts_3d[i].at(j).Y();
      z = pts_3d[i].at(j).Z();

      vcg::tri::Allocator<MyMesh>::AddVertex(m, MyMesh::CoordType(x,y,z));
      m.vert[count++].SetS();
    }
  }

  DrawProgressBar(40, 1);
  vcg::tri::UpdateColor<MyMesh>::PerVertexConstant(m, vcg::Color4b::Red, true);
  std::cout<<"Vertices:"<<m.VN()<<std::endl;
  savePlyFileVcg(name,m);
}
/**
   Function returns K-nearest neighbors camera images for given search point
*/
int ImgIO::getKNNcamData(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointXYZ &searchPoint, const std::vector<std::string> &filenames, std::vector<cv::Mat> &out_imgs, int K){

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  int out = 0;
  
  if(kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)>0){
    for(int i = 0 ; i < K ; i++)
      out_imgs.push_back( getImg(filenames[pointIdxNKNSearch[i]]) );
    out = 1;
  }
  return out;
}

/**
   Function displays all the images given in the input vector.
*/
void ImgIO::dispImgs(const std::vector<cv::Mat>& inImgs){
  
  std::ostringstream tmpString;
  int moveFactor = 200;

  for(int i = 0 ; i < inImgs.size(); i++){
    tmpString<<i;

    cv::namedWindow(tmpString.str(), cv::WINDOW_NORMAL);
    cv::moveWindow(tmpString.str(), moveFactor, 0);
    cv::imshow(tmpString.str(), inImgs[i]);

    tmpString.flush();
    moveFactor+=moveFactor;
  }
  cv::waitKey(0);                   
}
/**
   Function extracts points from the binary change mask.
*/
void ImgIO::getPtsFromMask(const cv::Mat &mask, std::vector<cv::Point2f> &pts_vector){

  int rows = mask.rows;
  int cols = mask.cols;
  
  for(int r = 0; r < rows; r++){
    for(int c = 0; c < cols; c++){      
      if(mask.at<uchar>(r,c)>0){
	pts_vector.push_back(cv::Point2f(c,r));
      }
    }
  }
}

/**
   Function extracts rotation translation matrix [R | t] from the VCG shot structure into OpenCV Mat structure
*/
cv::Mat ImgIO::getRtMatrix(const vcg::Shot<float> &shot){


  cv::Mat mat_Rt(3,4, CV_64FC1);
  mat_Rt = cv::Mat::zeros(3,4, CV_64FC1);

  vcg::Matrix44f mat_rot = shot.GetWorldToExtrinsicsMatrix();
  
  for(int i = 0 ; i < 3 ; i++){
    for(int j = 0 ; j < 4 ; j++){
      mat_Rt.at<double>(i,j) = static_cast<double>(mat_rot[i][j]);
    }   
  }
 
  return mat_Rt;
}

/**
   Function extracts intrinsic matrix from VCG shot structure into OpenCV Mat structure.
*/
cv::Mat ImgIO::getIntrMatrix(const vcg::Shot<float> &shot){

  cv::Mat intr_mat;
  intr_mat = cv::Mat::zeros(3,3, CV_64FC1);

  intr_mat.at<double>(0,0) = shot.Intrinsics.FocalMm/shot.Intrinsics.PixelSizeMm[0];
  intr_mat.at<double>(1,1) = shot.Intrinsics.FocalMm/shot.Intrinsics.PixelSizeMm[1];
  intr_mat.at<double>(0,2) = shot.Intrinsics.CenterPx[0];
  intr_mat.at<double>(1,2) = shot.Intrinsics.CenterPx[1];
  intr_mat.at<double>(2,2) = 1;

  return intr_mat;
}

/**
   Function projects 2D change mask into 3-dimensional space using point cloud voxelization and computation of ray intersections with the voxels
*/
std::vector<vcg::Point3f> ImgIO::projChngMask(const std::string &filename, const cv::Mat &chng_mask, const vcg::Shot<float> &shot){
  
  std::cout<<"Projecting 2D change mask into 3D space..." <<std::endl;
  std::vector<vcg::Point3f> out_pts;
  std::vector<cv::Point2f> mask_pts;
  rayBox voxel_grid;
  Eigen::Vector4f origin;
  vcg::Point3f tmp_pt;
  double prog_perc = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  MeshIO::getPlyFilePCL(filename, cloud);

  shot.Extrinsics.Tra().ToEigenVector(cloud->sensor_origin_);

  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize (0.1f, 0.1f, 0.1f);
  voxel_grid.initializeVoxelGrid();
    
  getPtsFromMask(chng_mask, mask_pts);

  shot.Extrinsics.Tra().ToEigenVector(origin);  
    
  for(int i = 0 ; i < mask_pts.size(); i++){
    
    if(i % 100 == 0){
      prog_perc = double(i)/double(mask_pts.size());
      DrawProgressBar(40, prog_perc);
    }

    Eigen::Vector4f direction;
    vcg::Point3f tmp_dir = shot.UnProject(vcg::Point2f(mask_pts[i].x, mask_pts[i].y), 1000);

    tmp_dir.ToEigenVector(direction);

    float tmp_mp = voxel_grid.getBoxIntersection(origin, direction);
      
    if(tmp_mp == -1.0f){
      continue;
    }
    
    /*
      float mp_factor = 0.6;
      direction = origin + tmp_mp*direction;
      direction = origin + mp_factor*direction;  
      Eigen::Vector3i vox_coord = voxel_grid.getGridCoord(direction[0],direction[1],direction[2]);     
      int is_occ = 0;
    */
    
    int cloud_idx = -1;
    
    //    std::vector<Eigen::Vector3i> out_ray;
    //    voxel_grid.occlusionEstimation(is_occ, out_ray, vox_coord);    
    
    cloud_idx = voxel_grid.getFirstOccl(origin, direction, tmp_mp);
    
    if(cloud_idx!=-1){
      pcl::PointXYZ fin_pt = cloud->points[cloud_idx];
      
      tmp_pt = PclProcessing::pcl2vcgPt(fin_pt);    
      tmp_pt[0] += rand()%-1+1;
      out_pts.push_back(tmp_pt);
    }
  } 
  DrawProgressBar(40, 1);
  std::cout<<"\n";
  return out_pts;
}


/**
   Function projects 2D change mask into 3-dimensional space using triangulation.
*/
cv::Mat ImgIO::projChngMaskTo3D(const cv::Mat &chngMask, const vcg::Shot<float> &cam1, const vcg::Shot<float> &cam2, const cv::Mat &H){

  const clock_t begin_time = clock();
  
  std::vector<cv::Point2f> cam1_points, cam2_points;

  getPtsFromMask(chngMask, cam1_points);
  
  cv::Mat cam1_fmat;
  cv::Mat cam2_fmat;
 
  cv::Mat cam1_Rt = getRtMatrix(cam1);
  cv::Mat cam2_Rt = getRtMatrix(cam2);
  
  cv::Mat cam1_intr = getIntrMatrix(cam1);
  cv::Mat cam2_intr = getIntrMatrix(cam2);

  cam1_fmat = cam1_intr*cam1_Rt;
  cam2_fmat = cam2_intr*cam2_Rt;

  std::cout<<"Rt 1:\n"<<cam1_Rt<<"\n intr 1:\n"<<cam1_intr<<std::endl;

  cv::perspectiveTransform(cam1_points, cam2_points, H);  
 
  std::cout<<"P1 size:"<<cam1_points.size()<<" P2 size:"<<cam2_points.size()<<std::endl;
  std::cout<<"Sum of chng:"<<cv::sum(chngMask).val[0]/255<<std::endl;

  cv::Mat pnts3D(1, cam1_points.size(), CV_64FC4);

  cv::triangulatePoints(cam1_fmat, cam2_fmat, cam1_points, cam2_points, pnts3D); 
  
  std::cout << "Time:"<<float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
  std::cout << "No of 3D points:"<<pnts3D.size() << std::endl;
  
  return pnts3D;
}

/**
   Class responsible for Input/Output operations.
*/
ChangeDetectorIO::ChangeDetectorIO(std::vector<std::string> inVector){
  filenames.resize(inVector.size());
  filenames = inVector;
}

ChangeDetectorIO::ChangeDetectorIO(std::string inDir){
  filenames.push_back(inDir);
}
/**
   Function saves frames from the input video for given frequency(frameRate).
*/
void VidIO::saveImgFromVideo(std::string outDir, int frameRate){
  
  cv::VideoCapture vidCap(filenames[0]);
  cv::Mat tmpImage;
  
  std::cout<<"Saving frames from video file..."<<std::endl;
  if(vidCap.isOpened()){
    for(int i = 0;;i++){
      ostringstream ss;
      ss<<i;
      vidCap>>tmpImage;

      if(i%frameRate==0){
	if(tmpImage.empty())
	  break;
	cv::imwrite(outDir+ss.str()+".jpg",tmpImage);
      }
    }
  }
  std::cout<<"Done!"<<std::endl;
}


void CmdIO::callVsfm(std::string inCmd){
  
  std::string outCommand;
  std::string vsfmCommand("VisualSFM");
  outCommand = vsfmCommand+inCmd;
  
  system(outCommand.c_str());
}

void CmdIO::callCmd(std::string inCmd){
  system(inCmd.c_str());
}


void FileIO::readNewFiles(const std::string &list_filename, std::vector<std::string> &out_filenames){

  std::ifstream inFile(list_filename.c_str());
  std::string tmpString;

  while(getline(inFile,tmpString))
    out_filenames.push_back(tmpString);

  inFile.close();
}

/**
   Function converts cameras read from NVM file into VCG Shot objects. Part of the function is based on the function Open() in import_out.h in VCG library
*/
std::vector<vcg::Shot<float> > FileIO::nvmCam2vcgShot(const std::vector<CameraT> &camera_data, const std::vector<std::string> names){
  
  std::cout<<"Converting NVM Cam structure to VCG shot structure..."<<std::endl;

  std::vector<vcg::Shot<float> > outputShots;  
  std::size_t inSize = camera_data.size();
  vcg::Shot<float> tmpShot;
  outputShots.resize(inSize);
  
  float f;
  float R[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,1};
  vcg::Point3f t;
  CameraT tmpCam;
  int count;

  for(int i = 0 ; i < inSize ; i++){
    count = 0;

    tmpCam = camera_data[i];
    f = tmpCam.f;

    for(int j = 0; j < 3 ; j++){
      R[count] = tmpCam.m[j][0];
      R[count+1] = tmpCam.m[j][1];
      R[count+2] = tmpCam.m[j][2];
      count+=4;
    }
    
    t[0] = tmpCam.t[0];
    t[1] = tmpCam.t[1];
    t[2] = tmpCam.t[2];

    vcg::Matrix44f mat = vcg::Matrix44<vcg::Shotf::ScalarType>::Construct<float>(R);

    vcg::Matrix33f Rt = vcg::Matrix33f( vcg::Matrix44f(mat), 3);
    Rt.Transpose();

    vcg::Point3f pos = Rt * vcg::Point3f(t[0], t[1], t[2]);

    outputShots[i].Extrinsics.SetTra(vcg::Point3<vcg::Shotf::ScalarType>::Construct<float>(-pos[0],-pos[1],-pos[2]));
    outputShots[i].Extrinsics.SetRot(mat);
    outputShots[i].Intrinsics.FocalMm = f;
    outputShots[i].Intrinsics.k[0] = 0.0;
    outputShots[i].Intrinsics.k[1] = 0.0;
    outputShots[i].Intrinsics.PixelSizeMm = vcg::Point2f(1,1);
    
    cv::Mat image;
    image = cv::imread(names[i]);
    cv::Size size = image.size();

    outputShots[i].Intrinsics.ViewportPx = vcg::Point2i(size.width,size.height);
    outputShots[i].Intrinsics.CenterPx[0] = (int)((double)outputShots[i].Intrinsics.ViewportPx[0]/2.0f);
    outputShots[i].Intrinsics.CenterPx[1] = (int)((double)outputShots[i].Intrinsics.ViewportPx[1]/2.0f);
  }
  
  std::cout<<"Done."<<std::endl;
  return outputShots;
}

void FileIO::getNVM(std::string filename, std::vector<CameraT>& camera_data, std::vector<std::string>& names){

  std::ifstream inFile(filename.c_str());
  
  std::cout<<"Loading NVM file... ";
  if(LoadNVM(inFile, camera_data, names))
    std::cout<<"Done!"<<endl;
  inFile.close();
}



void dispProjPt(const vcg::Point2i &inPt, cv::Mat &inImg){
  
  static cv::Scalar color = cv::Scalar(0, 0, 0);	
    
  cv::circle(inImg, cv::Point(inPt.X(),inPt.Y()), 50 , color, 15);
  
  cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.
  cv::imshow( "Display window", inImg);                   // Show our image inside it.
      
  cv::waitKey(0);                        

}

void getImgSet(std::vector<std::string> fileDirs, std::vector<cv::Mat> &outImgSet){

  std::cout<<"Loading images..."<<std::endl;
  cv::Mat image;

  for(std::vector<std::string>::iterator it = fileDirs.begin(); it!=fileDirs.end(); ++it){
    image = cv::imread("/home/bheliom/Pictures/NotreDame/"+*it, CV_LOAD_IMAGE_COLOR);
    std::cout<<"/home/bheliom/Pictures/NotreDame/"+*it<<std::endl;

    if(!image.data)
      std::cout<<"Could not open the file!"<<std::endl;
    else
      outImgSet.push_back(image);
  }

  std::cout<<"Done."<<std::endl;
}

cv::Mat getImg(std::string fileDirs){
  
  return cv::imread(fileDirs.c_str());//, CV_LOAD_IMAGE_COLOR);
}

void readCmdInput(std::map<int,std::string> &inStrings, int argc, char** argv){
  
  int flags, opt;
  int nsecs, tfnd;
  
  nsecs = 0;
  tfnd = 0;
  flags = 0;
  
  while ((opt = getopt(argc, argv, "m:p:b:i:o:n")) != -1) {
    switch (opt) {
	
    case 'm':
      inStrings[MESH] = optarg;
      break;
    case 'p':
      inStrings[PMVS] = optarg;
      break;
    case 'b':
      inStrings[BUNDLER] = optarg;
      break;
    case 'i':
      inStrings[IMAGELIST] = optarg;
      break;
    case 'o':
      inStrings[OUTDIR] = optarg;
      break;
	
    default: /* '?' */
      fprintf(stderr, "Usage: %s [-m input mesh] [-p input PMVS] [-b input bundler file] [-i input image list]\n",
	      argv[0]);
    }
  }
}

void getPlyFileVcg(std::string filename, MyMesh &m){

  vcg::tri::io::ImporterPLY<MyMesh> importVar;
  
  if(importVar.Open(m,filename.c_str()))
    {
      printf("Error reading file  %s\n",filename.c_str());
      std::cout<<vcg::tri::io::ImporterOFF<MyMesh>::ErrorMsg(importVar.Open(m,filename.c_str()))<<std::endl;
      exit(0);
    }
  std::cout<<"Mesh loaded correctly. No. of faces:"<<m.FN()<<" no. of vertices:"<<m.VN()<<std::endl;
}

void savePlyFileVcg(std::string filename, MyMesh &m){
  
  vcg::tri::io::ExporterPLY<MyMesh> exportVar;

  exportVar.Save(m,filename.c_str(),vcg::tri::io::Mask::IOM_VERTCOLOR);

}

void getBundlerFile(MyMesh &m, std::string filename, std::string filename_images, std::vector<vcg::Shot<float> > &shots, std::vector<std::string> &image_filenames){

  std::cout<<"Start reading bundler file..."<<std::endl;
  vcg::tri::io::ImporterOUT<MyMesh> importVar;

  if(importVar.Open(m, shots , image_filenames, filename.c_str(), filename_images.c_str()))
    std::cout<<"Error reading the bundler file!"<<std::endl;

  std::cout<<"Done."<<std::endl;
}
