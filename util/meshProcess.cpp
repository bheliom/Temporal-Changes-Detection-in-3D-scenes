#include "meshProcess.hpp"

typedef vcg::tri::UpdateTopology<MyMesh>::PEdge SingleEdge;

/*Function returns average length of the edge in the whole mesh*/
double getEdgeAverage(MyMesh &m){

  vcg::tri::UpdateTopology<MyMesh>::FaceFace(m);
  
  std::vector<SingleEdge> edges;
 
  vcg::tri::UpdateTopology<MyMesh>::FillUniqueEdgeVector(m,edges,true);

  MyMesh::VertexPointer vp1,vp2;
  
  vcg::Point3f tmp;
  double tmpSum = 0;
  double edgesSize = edges.size();

  std::cout<<"No of edges: "<<edgesSize<<std::endl;

  for(int i = 0; i<edgesSize; i++){

    vp1 = edges[i].v[0];
    vp2 = edges[i].v[1];
    
    tmp = vp1->P() - vp2->P();
    
    tmpSum+=sqrt(tmp.dot(tmp));

  }

  return tmpSum/edgesSize;
}

/*Function returns edge average length for given face*/
double getFaceEdgeAverage(MyFace &f){
  
  vcg::Point3f tmp1,tmp2,tmp3;
  
  tmp1 = f.cP(0) - f.cP(1);
  tmp2 = f.cP(0) - f.cP(2);
  tmp3 = f.cP(1) - f.cP(2);
  
  return (sqrt(tmp1.dot(tmp1))+sqrt(tmp2.dot(tmp2))+sqrt(tmp3.dot(tmp3)))/3;    
}
