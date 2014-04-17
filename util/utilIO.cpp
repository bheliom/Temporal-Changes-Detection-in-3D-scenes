#include "utilIO.hpp"
#include "pbaDataInterface.h"
#include "../common/globVariables.hpp"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include "opencv2/calib3d/calib3d.hpp"

void ImgIO::dispImgs(const std::vector<cv::Mat>& inImgs){
  
  std::ostringstream tmpString;
  int moveFactor = 200;

  for(int i = 0 ; i < inImgs.size(); i++){
    tmpString<<i;
    cv::namedWindow(tmpString.str(), cv::WINDOW_NORMAL);
    cv::moveWindow(tmpString.str(), moveFactor, 0);
    cv::imshow(tmpString.str(), inImgs[i]);

    tmpString.clear();
    moveFactor+=moveFactor;
  }
  cv::waitKey(0);                   
}

void ImgIO::getPtsFromMask(const cv::Mat &mask, std::vector<cv::Point2f> &pts_vector){

int num_nonzero_elem = cv::sum(mask).val[0];
 pts_vector.resize(num_nonzero_elem);
  
 int count = 0;
  
 for(int r = 0; r < mask.rows; r++){
   for(int c = 0; c < mask.cols; c++){
     if(mask.at<int>(r,c)>0) pts_vector[count++] = cv::Point2f(c,r);
   }
 }
}

cv::Mat ImgIO::getRtMatrix(const vcg::Shot<float> &shot){

  cv::Mat mat_Rt(3,4, CV_64FC1);
  mat_Rt = cv::Mat::zeros(3,4, CV_64FC1);

  vcg::Matrix44f mat_rot = shot.Extrinsics.Rot();
  vcg::Point3f mat_tra = shot.Extrinsics.Tra();
  
  for(int i = 0 ; i < 3 ; i++){
    for(int j = 0 ; j < 3 ; j++){
      mat_Rt.at<int>(i,j) = mat_rot[i][j];
    }
    mat_Rt.at<int>(i,3) = mat_tra[i];
  }
  return mat_Rt;
}

cv::Mat ImgIO::getIntrMatrix(const vcg::Shot<float> &shot){

  cv::Mat intr_mat;
  intr_mat = cv::Mat::zeros(3,3, CV_64FC1);

  intr_mat.at<int>(0,0) = shot.Intrinsics.FocalMm;
  intr_mat.at<int>(1,1) = shot.Intrinsics.FocalMm;
  intr_mat.at<int>(0,2) = shot.Intrinsics.CenterPx[0];
  intr_mat.at<int>(1,2) = shot.Intrinsics.CenterPx[1];
  intr_mat.at<int>(2,2) = 1;

  return intr_mat;
}

void ImgIO::projChngMaskTo3D(cv::Mat chngMask, vcg::Shot<float> cam1, vcg::Shot<float> cam2, cv::Mat F, std::vector<cv::Point2f> &cam1_points, std::vector<cv::Point2f> &cam2_points){

  //  std::vector<cv::Point2f> cam1_points;
  //  std::vector<cv::Point2f> cam2_points;
  
  getPtsFromMask(chngMask, cam1_points);
  cam2_points.resize(cam1_points.size());

  cv::Mat cam1_fmat;
  cv::Mat cam2_fmat;
 
  cv::Mat cam1_Rt = getRtMatrix(cam1);
  cv::Mat cam2_Rt = getRtMatrix(cam2);
  
  cv::Mat cam1_intr = getIntrMatrix(cam1);
  cv::Mat cam2_intr = getIntrMatrix(cam2);

  cam1_fmat = cam1_intr*cam1_Rt;
  cam2_fmat = cam2_intr*cam2_Rt;
  
  cv::perspectiveTransform(cam1_points, cam2_points, F);
  
  
  cv::Point2f tmpPoint;
  
  //In case if we need to remove negative values
  
  std::cout<<"Jestem tutaj"<<std::endl;
  for(int i = 0; i < cam1_points.size(); i++){
    tmpPoint = cam2_points.at(i);
    if(tmpPoint.x<0 || tmpPoint.y<0){
      cam1_points.erase(cam1_points.begin()+i);
      cam2_points.erase(cam2_points.begin()+i);
    }
  }
  std::cout<<"teraz jestem tutaj"<<std::endl;
  
  // cv::Mat pnts3D(1,cam1_points.size(),CV_64FC4);
  //  cv::triangulatePoints(cam1_fmat, cam2_fmat, cam1_points, cam2_points, pnts3D);
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

void VidIO::saveImgFromVideo(std::string outDir,int frameRate){
  
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

void inputHandler(std::vector<std::string> inputStrings){

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
