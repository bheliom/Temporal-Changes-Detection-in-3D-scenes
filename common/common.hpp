#ifndef COMMON_H
#define COMMON_H

#include "globVariables.hpp"

#include<vcg/math/shot.h>
#include<wrap/io_trimesh/import_out.h>
#include<vcg/complex/complex.h>
#include<wrap/io_trimesh/import.h>
#include<wrap/io_trimesh/export_ply.h>
#include<vcg/complex/algorithms/update/topology.h>
#include<vcg/complex/algorithms/update/normal.h>
#include<vcg/complex/algorithms/update/color.h>

class MyVertex; class MyEdge; class MyFace;

struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex>   ::AsVertexType,
  vcg::Use<MyEdge>     ::AsEdgeType,
  vcg::Use<MyFace>     ::AsFaceType>{};
  
class MyVertex  : public vcg::Vertex< MyUsedTypes, vcg::vertex::Color4b, vcg::vertex::Coord3f, vcg::vertex::VFAdj, vcg::vertex::Normal3f, vcg::vertex::BitFlags  >{};
class MyFace    : public vcg::Face<   MyUsedTypes, vcg::face::Color4b, vcg::face::FFAdj, vcg::face::VFAdj, vcg::face::VertexRef, vcg::face::BitFlags > {};
class MyEdge    : public vcg::Edge<   MyUsedTypes> {};
  
class MyMesh    : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> , std::vector<MyEdge>  > {};

struct PtCamCorr{
  vcg::Point3f pts_3d;
  std::vector<cv::Point2i> feat_coords;
  std::vector<int> ptidx;
  std::vector<int> camidx;
  cv::Point3i ptc;
};

struct ImgFeature{
  int idx;
  int x;
  int y;

  ImgFeature(){}

  ImgFeature(int in_idx,int in_x, int in_y){
    idx = in_idx;
    x = in_x;
    y = in_y;
  }
};

#endif









