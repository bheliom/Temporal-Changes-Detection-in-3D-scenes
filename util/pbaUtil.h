/*
This file originaly from Changchang Wu has been modified for purpose of this project
 */
#ifndef _PBAUTIL_H_
#define _PBAUTIL_H_

////////////////////////////////////////////////////////////////////////////
//	File:		    util.h
//	Author:		    Changchang Wu (ccwu@cs.washington.edu)
//	Description :   some utility functions for reading/writing SfM data
//
//  Copyright (c) 2011  Changchang Wu (ccwu@cs.washington.edu)
//    and the University of Washington at Seattle 
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  Version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <math.h>
#include <time.h>
#include <iomanip>
#include <algorithm>

#include "../common/common.hpp"
#include "pbaDataInterface.h"
#include "utilIO.hpp"

using namespace std;

/////////////////////////////////////////////////////////////////////////////

bool LoadNVM(ifstream& in, vector<CameraT>& camera_data, vector<string>& names, vector<PtCamCorr>& pt_corr, map<int, vector<ImgFeature> >&);

#endif
