#include "chngDet.hpp"
#include "../util/meshProcess.hpp"

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_adjacency_container.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

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

std::vector<int> ImgChangeDetector::imgFeatDiff(const std::vector<ImgFeature>& new_imgs_feat, const std::vector<ImgFeature>& old_imgs_feat, const std::vector<PtCamCorr>& pts_corr, const std::set<int>& new_imgs_idx, const std::set<int>& old_imgs_idx){

  std::vector<int> out_pts;
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
      out_pts.push_back(new_imgs_feat[i].idx);    
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
      out_pts.push_back(old_imgs_feat[i].idx);
  }

  return out_pts;
}

typedef pcl::octree::OctreeContainerPointIndices LeafContainerT;
typedef pcl::PointXYZRGBA PointT;

void MeshChangeDetector::energyMinimization(pcl::PointCloud<PointT>::Ptr old_cloud, pcl::PointCloud<PointT>::Ptr chng_mask){

  PclProcessing::changeCloudColor(*old_cloud, 255, 255, 255);
  PclProcessing::changeCloudColor(*chng_mask, 255, 0, 0);

  pcl::PointCloud<PointT>::Ptr merged_cloud(new pcl::PointCloud<PointT>);
  *merged_cloud = (*old_cloud)+(*chng_mask);

  float resolution = 0.1f;
  MyOctree octree(resolution);
 
  octree.setInputCloud (merged_cloud);
  octree.addPointsFromInputCloud ();

  LeafContainerT *leaf_container;
  typename MyOctree::LeafNodeIterator leaf_itr;

  for(leaf_itr = octree.leaf_begin(); leaf_itr!=octree.leaf_end() ; ++leaf_itr){
    int red_no = 0;
    
    leaf_itr.getNodeID();
    
    std::vector<int> pts_idx;
    
    pcl::octree::OctreeKey key_arg = leaf_itr.getCurrentOctreeKey ();
    leaf_container = &(leaf_itr.getLeafContainer());
    leaf_container->getPointIndices(pts_idx);

    red_no = getRedCount(pts_idx, *merged_cloud);

    if(red_no!=0)
      std::cout<<"no of red: "<<red_no<<"no total:"<<pts_idx.size()<<std::endl;

    std::vector<LeafContainerT*> tmp_vec;
    octree.findNeighbors(key_arg, tmp_vec);    
  }
} 

int MeshChangeDetector::getRedCount(const std::vector<int> &pts_idx, const pcl::PointCloud<pcl::PointXYZRGBA>& in_cloud){
  
  int count = 0;
  for(int i = 0 ; i<pts_idx.size(); i++)
    if(in_cloud.points[pts_idx[i]].getRGBVector3i()[1]==0){
      count++;
    }  
  return count;
}

void MyOctree::findNeighbors(const pcl::octree::OctreeKey& key_arg, std::vector<LeafContainerT*>& out_vec){

  pcl::octree::OctreeKey neighbor_key;

  for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
	{
	  for (int dz = -1; dz <= 1; ++dz)
	    {
	      neighbor_key.x = key_arg.x + dx;
	      neighbor_key.y = key_arg.y + dy;
	      neighbor_key.z = key_arg.z + dz;
	      LeafContainerT * neighbor = this->findLeaf(neighbor_key.x, neighbor_key.y, neighbor_key.z);
	      if (neighbor)
		{
		  out_vec.push_back(neighbor);
		}
	    }
	}
    }
}
