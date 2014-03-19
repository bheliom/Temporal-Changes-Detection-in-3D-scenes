#ifndef MESHPROCESS_H
#define MESHPROCESS_H

#include "../common/common.hpp"

double getEdgeAverage(MyMesh &m);
double getFaceEdgeAverage(MyFace &f);
void removeUnnFaces(MyMesh &m, int thresVal);

#endif
