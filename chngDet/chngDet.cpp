

#include "chngDet.hpp"

cv::Mat ImgChangeDetector::getImageDifference(cv::Mat img1, cv::Mat img2){
  return cv::abs(img1 - img2);
}

std::vector<vcg::Point3f> ImgChangeDetector::projChngMask(cv::Mat chngMask, vcg::Shot<float> inShot){
  
  std::vector<vcg::Point3f> outVec;
  /*
  for(int i = 0; chngMask.rows ; i++)
    for(int j = 0; chngMask.cols ; j++)
      if(chngMask.at(i,j)>0)
	//	outVec.push_back(inShot.Intr
	*/
  return outVec;   
}

void ImgChangeDetector::imgDiffThres(cv::Mat im1, cv::Mat im2, cv::Mat H, cv::Mat &mask){
  
  cv::Mat im1_trans;

  warpPerspective(im1, im1_trans, H, im1.size());
      
  cv::Mat diffImg(cv::abs(im2-im1_trans));  
  cv::Mat outImgG(im2.clone());

  warpPerspective(diffImg, outImgG, H, diffImg.size(), cv::WARP_INVERSE_MAP);
  
  cv::Mat finThres;
  cv::Mat finMask;

  cv::cvtColor(outImgG, finThres, CV_BGR2GRAY);      
  cv::threshold(finThres, mask, 30, 255, CV_THRESH_OTSU);

}
