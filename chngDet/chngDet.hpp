
#ifndef _CHNGDET_H_
#define _CHNGDET_H_

#include<vcg/space/point3.h>
#include "../common/common.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <set>
class ChangeDetector{

protected:
  MyMesh m;

public:
  void setInMesh(MyMesh inMesh)
  {vcg::tri::Append<MyMesh,MyMesh>::MeshCopy(m,inMesh);}

  virtual std::vector<float> getPofChange()=0;
  virtual std::vector<vcg::Point3f> getChangeMap()=0;

};

class ImgChangeDetector : public ChangeDetector{

protected:
  std::vector<std::string> newImgFilenames;

public:
  void setNewImgFilenames(std::vector<std::string> filenames)
  {newImgFilenames = filenames;}
  cv::Mat getImageDifference(cv::Mat img1, cv::Mat img2);

  std::vector<vcg::Point3f> projChngMask(cv::Mat chngMask, vcg::Shot<float> inShot);
  
  std::vector<float> getPofChange(){
  }
  std::vector<vcg::Point3f> getChangeMap(){
  }

  static void imgDiffThres(cv::Mat, cv::Mat, cv::Mat, cv::Mat&);
  
  static std::vector<vcg::Point3f> imgFeatDiff(const std::vector<ImgFeature>&, const std::vector<ImgFeature>&, const std::vector<PtCamCorr>&, const std::set<int>&);
};

class MeshChangeDetector : public ChangeDetector{

public:
  std::vector<float> getPofChange(){
  }
  std::vector<vcg::Point3f> getChangeMap(){
  }
  
};

class LaserChangeDetector : public ChangeDetector{

public:

  std::vector<float> getPofChange(){
  }
  std::vector<vcg::Point3f> getChangeMap(){
  }

};

class MpImgChangeDetector : public ImgChangeDetector{

public:
  std::vector<float> getPofChange(){
  }
  std::vector<vcg::Point3f> getChangeMap(){
  }

};

#endif
