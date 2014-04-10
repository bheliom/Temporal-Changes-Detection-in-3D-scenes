#include <fstream>
#include <map>
#include <boost/algorithm/string.hpp>
#include "meshProcess.hpp"

typedef vcg::tri::UpdateTopology<MyMesh>::PEdge SingleEdge;

/**
Function splits given string depending on defined delimeter
 */
std::vector<std::string> FileProcessing::split(const std::string &inString, char delim){

  std::vector<std::string> outStrings;
  std::stringstream ss(inString);
  std::string item;

  while(std::getline(ss, item, delim))
	outStrings.push_back(item);

  return outStrings;

}

/**
Function processes NVM file created after calling VisualSfM with new images and old model. It creates new NVM file consisting of camera positions for new images exclusively so that loadNVM function can be used.
*/
void FileProcessing::procNewNVMfile(const std::string &nvmFileDir, const std::vector<std::string> &imgFilenames, const std::string &outName){
  
  std::cout<<"Processing new NVM file..."<<std::endl;
  std::string tmpString;
  std::map<std::string, std::string> tmpMap;
  std::vector<std::string> tmpStrVec;

  std::ofstream outFile;
  outFile.open(outName.c_str());
  
  std::ifstream inFile(nvmFileDir.c_str());  

  //Create map with filenames
  for(int i = 0 ; i < imgFilenames.size() ; i++)
    tmpMap[imgFilenames[i]] = "";  
  
  //Get the header
  inFile>>tmpString;
  outFile<<tmpString+"\n\n";

  //Number of new cameras
  outFile<<imgFilenames.size()<<"\n\n";

  while(std::getline(inFile, tmpString))
    {      
      boost::split(tmpStrVec, tmpString, boost::is_any_of(" \t"));
      
      if(tmpMap.find(tmpStrVec[0])!=tmpMap.end())
	outFile << tmpString + "\n";      
      tmpStrVec.clear();
    }
  outFile.close();

 std::cout<<"Done."<<std::endl;

}

pcl::PointXYZ vcg2pclPt(vcg::Point3<float> inPt)
{
  pcl::PointXYZ outPt;
  outPt.x = inPt.X();
  outPt.y = inPt.Y();
  outPt.z = inPt.Z();
  
  return outPt;
}

/**Function calculates image coordinates of the point projected using vcg::Shot class member function*/
vcg::Point2i getPtImgCoord(const vcg::Point2f &inPoint, const vcg::Shot<float> &inShot){
  
  vcg::Point2i tmpPoint(static_cast<int>(inPoint.X()), static_cast<int>(inPoint.Y()));

  return tmpPoint;
}


void findOcc(std::map<int, int> &inMap, std::vector<int> &outVector, int noOfOut){
  int tmpMax = 0;
  int tmpMaxId = 0;
  
  if(inMap.size() < noOfOut){
    for (std::map<int,int>::iterator it = inMap.begin(); it != inMap.end(); it++)
      outVector.push_back(it->first);
  }
  
  else {
    for (int k = 0; k < noOfOut; k++){
      for (std::map<int,int>::iterator it = inMap.begin(); it != inMap.end(); it++)
	if (tmpMax<it->second) {
	  tmpMax = it->second;
	  tmpMaxId = it->first;
	}
      outVector.push_back(tmpMaxId);
      inMap.erase(inMap.find(tmpMaxId));		
      tmpMax = 0;
      tmpMaxId = 0;
    }
  }
}

      /**Function returns average length of the edge in the whole mesh*/
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

    /**Function returns edge average length for given face*/
double getFaceEdgeAverage(MyFace &f){
  
  vcg::Point3f tmp1,tmp2,tmp3;
  
  tmp1 = f.cP(0) - f.cP(1);
  tmp2 = f.cP(0) - f.cP(2);
  tmp3 = f.cP(1) - f.cP(2);
  
  return (sqrt(tmp1.dot(tmp1))+sqrt(tmp2.dot(tmp2))+sqrt(tmp3.dot(tmp3)))/3;    
}

/**Function removes faces basing on the face average edge length*/
void removeUnnFaces(MyMesh &m, int thresVal){
  
  double avgEdge, tmp;
  MyMesh::FaceIterator fi;

  avgEdge = getEdgeAverage(m);

  std::cout<<"Edge average length: "<<avgEdge<<std::endl;
  
  vcg::tri::UpdateTopology<MyMesh>::VertexFace(m); 
  
  for(fi=m.face.begin(); fi!=m.face.end(); ++fi){
    tmp = getFaceEdgeAverage(*fi);

    if(tmp>thresVal*avgEdge)
      vcg::tri::Allocator<MyMesh>::DeleteFace(m, *fi);
  }  

}
