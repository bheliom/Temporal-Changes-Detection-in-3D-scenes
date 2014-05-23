#include "chngDet.hpp"
#include "../util/meshProcess.hpp"
#include "../maxflowLib/graph.h"

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_adjacency_container.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <map>
#include <cmath>
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
  
  for(int i = 0 ; i < new_imgs_feat.size() ; i++){
    PtCamCorr tmp_corr = pts_corr[new_imgs_feat[i].idx];
    bool add = false;
  
    if(tmp_corr.camidx.size()<=new_imgs_idx.size()){
      add = true;
      for(int j = 0 ; j < tmp_corr.camidx.size(); j++)
	if(old_imgs_idx.find(tmp_corr.camidx[j])!=old_imgs_idx.end())
	  add = false;      
    }

    if(add)
      out_pts.push_back(new_imgs_feat[i].idx);    
  }
    
  for(int i = 0 ; i < old_imgs_feat.size() ; i++){
    PtCamCorr tmp_corr = pts_corr[old_imgs_feat[i].idx];
    bool add = false;    

    if(tmp_corr.camidx.size()<=old_imgs_idx.size()){
      add = true;
      for(int j = 0 ; j < tmp_corr.camidx.size(); j++)
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
/**
This function uses MRF approach and grapcuts for the energy minimazation problem in order to reduce noise in the output
*/
void MeshChangeDetector::energyMinimization(pcl::PointCloud<PointT>::Ptr old_cloud, pcl::PointCloud<PointT>::Ptr chng_mask, double resolution){
  
  CmdIO::callCmd("rm change_mask_MRF.ply");
  int m = 1000;
  pcl::PointCloud<PointT>::Ptr copy1 = old_cloud;
  pcl::PointCloud<PointT>::Ptr copy2 = chng_mask;

  //Binarize input point clouds
  PclProcessing::changeCloudColor(*old_cloud, 255, 255, 255);
  PclProcessing::changeCloudColor(*chng_mask, 255, 0, 0);

  //Merge input point clouds
  pcl::PointCloud<PointT>::Ptr merged_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr merged_copy(new pcl::PointCloud<PointT>);
  *merged_cloud = (*old_cloud)+(*chng_mask);
  *merged_copy = (*copy1)+(*copy2);

  //Map connecting leaf indeces with their octree key
  std::map<std::string, int> map_leafs;

  //Create octree
  MyOctree octree(resolution);
  octree.setInputCloud (merged_cloud);
  octree.addPointsFromInputCloud ();

  LeafContainerT *leaf_container;
  typename MyOctree::LeafNodeIterator leaf_itr;

  int no_of_nodes = octree.getLeafCount();
  int avg_chng_pts_no = chng_mask->points.size()/no_of_nodes;
  int avg_white_pts = old_cloud->points.size()/no_of_nodes;

  //Create graphcut solver instance
  typedef Graph<int,int,int> GraphType;
  GraphType *g = new GraphType(/*estimated # of nodes*/ no_of_nodes, /*estimated # of edges*/ no_of_nodes*26); 

  int count = 0;

  //Iterate through octree to fill the map_leafs structure and add nodes to the solver
  for(leaf_itr = octree.leaf_begin(); leaf_itr!=octree.leaf_end() ; ++leaf_itr){

    pcl::octree::OctreeKey key_arg = leaf_itr.getCurrentOctreeKey();
    std::ostringstream ss;
    ss << key_arg.x << "-"<< key_arg.y<<"-"<<key_arg.z;
    map_leafs[ss.str()]=count;
    g->add_node();
    count++;
  }
  
  count = 0;

  //Iterate through octree leaves/nodes creating the graph for solver
  for(leaf_itr = octree.leaf_begin(); leaf_itr!=octree.leaf_end() ; ++leaf_itr){

    int red_no = 0;   
    std::vector<int> pts_idx;    
    
    //Get octree key for current node
    pcl::octree::OctreeKey key_arg = leaf_itr.getCurrentOctreeKey ();
    
    //Get leaf container for current node
    leaf_container = &(leaf_itr.getLeafContainer());
    
    //Get indices of points belonging to the node
    leaf_container->getPointIndices(pts_idx);
    
    //Count the number of change(red) points
    red_no = getRedCount(pts_idx, *merged_cloud);
    
    //Get octree keys of neighbors and their leafcontainers
    std::vector<LeafContainerT*> tmp_vec;
    std::vector<pcl::octree::OctreeKey> tmp_vec2;
    octree.findNeighbors(key_arg, tmp_vec);    
    octree.findNeighbors(key_arg, tmp_vec2);
  
    int weight_count = 0;
    
    //Iterate through node neighbors
    for(int i = 0 ; i < tmp_vec.size(); i ++){
    
      //Get current neighbor points indices
      std::vector<int> neigh_idx;
      tmp_vec[i]->getPointIndices(neigh_idx);
      
      //Count the number of change(red) points in the neighbor node
      int red_n_no = getRedCount(neigh_idx, *merged_cloud);
      weight_count+=red_n_no;
      
      //Get graph index of the neighbor node using the map
      pcl::octree::OctreeKey neigh_key = tmp_vec2[i];
      std::ostringstream ss;
      ss << neigh_key.x << "-"<< neigh_key.y<<"-"<<neigh_key.z;
      int neigh_idx_single = map_leafs[ss.str()];

      int coeff = avg_chng_pts_no;
      int coeff2 = abs(red_no - red_n_no);

      if(neigh_idx_single!=count){
	int result = avg_chng_pts_no;
	double exp_result = static_cast<double>(m)*static_cast<double>(exp(-(abs(red_no-red_n_no))));
	result = static_cast<int>(exp_result);

	g->add_edge(count, neigh_idx_single, result , result);

	/*	
	if(red_no!=red_n_no){
	  if(red_no == 0 || red_n_no == 0)
	    g->add_edge(count, neigh_idx_single, 0 , 0);
	  else
	    g->add_edge(count, neigh_idx_single, red_n_no, red_no);
	}
	else
	  if(red_no>0)
	    g->add_edge(count,neigh_idx_single, red_n_no, red_no);
	  else
	    g->add_edge(count, neigh_idx_single, avg_chng_pts_no, avg_chng_pts_no);
	*/
      }
    }
    //Add SOURCE/SINK weights for current node

    g->add_tweights(count, 100*(pts_idx.size()-red_no), 100*red_no);

    count++;
  }

  std::vector<std::vector<vcg::Point3f> > out_points;
  std::vector<vcg::Point3f> tmp_vcg_pts;

  count = 0;
  g->maxflow();

  for(leaf_itr = octree.leaf_begin(); leaf_itr!=octree.leaf_end() ; ++leaf_itr){
    if(g->what_segment(count) == GraphType::SINK){      
      std::vector<int> pts_idx;
      leaf_container = &(leaf_itr.getLeafContainer());
      leaf_container->getPointIndices(pts_idx);
      
      for(int i = 0 ; i<pts_idx.size();i++)
	tmp_vcg_pts.push_back(PclProcessing::pcl2vcgPt(merged_cloud->points[pts_idx[i]]));
    }
    count++;
  }
  
  std::cout<<"Old mask size: "<<chng_mask->points.size()<<" New mask size: "<<tmp_vcg_pts.size()<<std::endl;

  out_points.push_back(tmp_vcg_pts);
  std::vector<vcg::Color4b> pts_color(0);
  MeshIO::saveChngMask3d(out_points, pts_color, "change_mask_MRF.ply");

  delete g;
} 

int MeshChangeDetector::getRedCount(const std::vector<int> &pts_idx, const pcl::PointCloud<pcl::PointXYZRGBA>& in_cloud){
  
  int count = 0;
  for(int i = 0 ; i<pts_idx.size(); i++)
    if(in_cloud.points[pts_idx[i]].getRGBVector3i()[1]==0){
      count++;
    }  
  return count;
}
void MyOctree::findNeighbors(const pcl::octree::OctreeKey& key_arg, std::vector<pcl::octree::OctreeKey>& out_vec){

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
		  out_vec.push_back(neighbor_key);
		}
	    }
	}
    }
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
