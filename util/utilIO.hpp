#ifndef __UTILIO_H_INCLUDED__
#define __UTILIO_H_INCLUDED__

#include <string>
#include <unistd.h>
#include <map>

#include "../common/common.hpp"
#include<wrap/io_trimesh/import_off.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pbaUtil.h"

struct PtCamCorrespondence{
  std::vector<vcg::Point3f> pts_3d;
  std::vector<cv::Point2i> feat_coords;
  std::vector<int> ptidx;
  std::vector<int> camidx;
  std::vector<int> ptc;

};

/*
To run VisualSFM:

VisualSFM sfm+resume+fixcam <fullPathInput>/input.nvm <fullPathOutput>/output.nvm

It requires an image list called [input].nvm.txt with new images paths
*/

class ChangeDetectorIO{

protected:
  std::vector<std::string> filenames;
public:
  ChangeDetectorIO(std::vector<std::string>);
  ChangeDetectorIO(std::string);
  ChangeDetectorIO();
};

class MeshIO : public ChangeDetectorIO{

public:
  MeshIO() : ChangeDetectorIO(){};
  static void saveChngMask3d(const std::vector<std::vector<vcg::Point3f> >&, const std::string&);

  template <typename T>
  static void getPlyFilePCL(const std::string filename, boost::shared_ptr<pcl::PointCloud<T> > outCloud){  
    if(pcl::io::loadPLYFile<T> (filename, *outCloud) == -1)
      PCL_ERROR ("Couldn't read file\n"); 
  }
  
};

class ImgIO : public ChangeDetectorIO{

protected:

public:
  ImgIO() : ChangeDetectorIO(){};
  static void dispImgs(const std::vector<cv::Mat>&);
  static void getPtsFromMask(const cv::Mat&, std::vector<cv::Point2f>&);
  static cv::Mat projChngMaskTo3D(const cv::Mat&, const vcg::Shot<float>&, const vcg::Shot<float>&, const cv::Mat&);
  static std::vector<vcg::Point3f> projChngMask(const std::string&, const cv::Mat&, const vcg::Shot<float>&);
  static cv::Mat getRtMatrix(const vcg::Shot<float>&);
  static cv::Mat getIntrMatrix(const vcg::Shot<float>&);
  static int getKNNcamData(const pcl::KdTreeFLANN<pcl::PointXYZ>&, pcl::PointXYZ&, const std::vector<std::string>&, std::vector<cv::Mat>&, int K, std::vector<int>&);

  static std::vector<vcg::Point3f> projChngMaskCorr(const cv::Mat&, const std::vector<ImgFeature>&, const std::vector<PtCamCorr>&);

};

class VidIO : public ChangeDetectorIO{

public:
  VidIO(std::string inVid) : ChangeDetectorIO(inVid){};
  void saveImgFromVideo(std::string,int);

};

class FileIO : public ChangeDetectorIO{

public:
  FileIO(std::string inFile) : ChangeDetectorIO(inFile){};
  static void getNVM(std::string filename, std::vector<CameraT>& camera_data, std::vector<std::string>& names,std::vector<PtCamCorr>&, std::map<int, std::vector<ImgFeature> >& in_map); 
  static std::vector<vcg::Shot<float> > nvmCam2vcgShot(const std::vector<CameraT> &camera_data, const std::vector<std::string> names);
  static void readNewFiles(const std::string&, std::vector<std::string>&);

};

class CmdIO : public ChangeDetectorIO{

public:
  CmdIO(std::string inDirs) : ChangeDetectorIO(inDirs){};
  static void callVsfm(std::string);
  static void callCmd(std::string);
};

void dispProjPt(const vcg::Point2i &inPt, cv::Mat &inImg);

void getImgSet(std::vector<std::string> fileDirs, std::vector<cv::Mat> &outImgSet);

cv::Mat getImg(std::string fileDir);

void readCmdInput(std::map<int,std::string> &inputStrings, int argc, char** argv);

void getPlyFileVcg(std::string filename, MyMesh &m);

void savePlyFileVcg(std::string filename, MyMesh &m);

void getBundlerFile(std::string filename);

void getBundlerFile(MyMesh &m, std::string filename, std::string filename_images, std::vector<vcg::Shot<float> > &shots, std::vector<std::string> &image_filenames);

template <typename T>
void getPlyFilePCL(std::string filename, boost::shared_ptr<pcl::PointCloud<T> > outCloud){

if(pcl::io::loadPLYFile<T> (filename, *outCloud) == -1)
  PCL_ERROR ("Couldn't read file\n"); 
}

/**
 Function taken from http://stackoverflow.com/questions/60221/how-to-animate-the-command-line
 */

inline void DrawProgressBar(int len, double percent) {
  std::cout << "\x1B[2K"; // Erase the entire current line.
  std::cout << "\x1B[0E"; // Move to the beginning of the current line.
  std::string progress;
  for (int i = 0; i < len; ++i) {
    if (i < static_cast<int>(len * percent)) {
      progress += "=";
    } else {
      progress += " ";
    }
  }

  std::cout << "[" << progress << "] " << (static_cast<int>(100 * percent)) << "%";
  std::flush(std::cout);
}

#endif
