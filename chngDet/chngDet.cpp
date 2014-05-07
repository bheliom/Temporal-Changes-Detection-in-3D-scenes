

#include "chngDet.hpp"

cv::Mat ImgChangeDetector::getImageDifference(cv::Mat img1, cv::Mat img2){
  return cv::abs(img1 - img2);
}

std::vector<vcg::Point3f> ImgChangeDetector::projChngMask(cv::Mat chngMask, vcg::Shot<float> inShot){
  
  std::vector<vcg::Point3f> outVec;

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

std::vector<vcg::Point3f> ImgChangeDetector::imgFeatDiff(const std::vector<ImgFeature>& new_imgs_feat, const std::vector<ImgFeature>& old_imgs_feat, const std::vector<PtCamCorr>& pts_corr, const std::set<int>& new_imgs_idx, const std::set<int>& old_imgs_idx){

  std::vector<vcg::Point3f> out_pts;
  bool add = false;

  for(int i = 0 ; i < new_imgs_feat.size() ; i++){
    PtCamCorr tmp_corr = pts_corr[new_imgs_feat[i].idx];
    
    if(tmp_corr.camidx.size()<=new_imgs_idx.size()){
      add = true;
      for(int j = 0 ; j < tmp_corr.camidx.size()/2 ; j++)
	if(new_imgs_idx.find(tmp_corr.camidx[j])==new_imgs_idx.end())
	  add = false;      
    }

    if(add)
      out_pts.push_back(tmp_corr.pts_3d);    
  }
  
  add = false;

  for(int i = 0 ; i < old_imgs_feat.size() ; i++){
    PtCamCorr tmp_corr = pts_corr[old_imgs_feat[i].idx];
    
    if(tmp_corr.camidx.size()<=old_imgs_idx.size()){
      add = true;
      for(int j = 0 ; j <tmp_corr.camidx.size(); j++)
	if(new_imgs_idx.find(tmp_corr.camidx[j])!=new_imgs_idx.end())
	  add = false;      
    }
    
    if(add)
      out_pts.push_back(tmp_corr.pts_3d);
  }

  return out_pts;
}
