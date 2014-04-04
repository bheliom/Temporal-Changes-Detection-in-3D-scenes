
#ifndef _CHNGDET_H_
#define _CHNGDET_H_

#include<vcg/space/point3.h>
#include "../common/common.hpp"

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
/*
-nvm workspace directory
-IO for calling VisualSfM to get new bundler.out
-function to process new bundler.out just to get camera positions
 */
public:
void setNewImgFilenames(std::vector<std::string> filenames)
{newImgFilenames = filenames;}

std::vector<float> getPofChange(){
}
std::vector<vcg::Point3f> getChangeMap(){
}

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
