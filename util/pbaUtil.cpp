#include "pbaUtil.h"

using namespace std;

bool LoadNVM(ifstream& in, vector<CameraT>& camera_data, vector<string>& names, vector<PtCamCorr>& pt_corr, map<int,vector<ImgFeature> >& in_map)
{
    int rotation_parameter_num = 4; 
    bool format_r9t = false;
    string token;
    if(in.peek() == 'N') 
    {
        in >> token; //file header
        if(strstr(token.c_str(), "R9T"))
        {
            rotation_parameter_num = 9;    //rotation as 3x3 matrix
            format_r9t = true;
        }
    }
    
    int ncam = 0, npoint = 0, nproj = 0;   
    // read # of cameras
    in >> ncam;  if(ncam <= 1) return false; 

    //read the camera parameters
    camera_data.resize(ncam); // allocate the camera data
    names.resize(ncam);
    
    for(int i = 0; i < ncam; ++i)
     {
        double f, q[9], c[3], d[2];
        in >> token >> f ;
        for(int j = 0; j < rotation_parameter_num; ++j) in >> q[j]; 
        in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];

        camera_data[i].SetFocalLength(f);
        if(format_r9t) 
        {
            camera_data[i].SetMatrixRotation(q);
            camera_data[i].SetTranslation(c);
        }
        else
        {
            //older format for compability
            camera_data[i].SetQuaternionRotation(q);        //quaternion from the file
            camera_data[i].SetCameraCenterAfterRotation(c); //camera center from the file
        }
        camera_data[i].SetNormalizedMeasurementDistortion(d[0]); 
        names[i] = token;
    }


    in >> npoint;
    if(npoint <= 0){
      std::cout << ncam << " new cameras\n";
      return true; 
    }


    
    //read image projections and 3D points.
    pt_corr.resize(npoint); 
    for(int i = 0; i < npoint; ++i)
      {
	PtCamCorr tmp_corr;

	float pt[3]; int cc[3], npj;
	in  >> pt[0] >> pt[1] >> pt[2] 
	    >> cc[0] >> cc[1] >> cc[2] >> npj;
	
	for(int j = 0; j < npj; ++j)
	  {	   
	    int cidx, fidx; float imx, imy;
	    in >> cidx >> fidx >> imx >> imy;
	    
	    tmp_corr.camidx.push_back(cidx);    //camera index
	    tmp_corr.ptidx.push_back(i);        //point index
	    
	    ImgFeature tmp_img_feat(i, imx, imy); 
	    in_map[cidx].push_back(tmp_img_feat);

	    //add a measurment to the vector
	    tmp_corr.feat_coords.push_back(cv::Point2i(imx, imy));
	    nproj ++;
	  }

	tmp_corr.pts_3d[0] = pt[0];
	tmp_corr.pts_3d[1] = pt[1];
	tmp_corr.pts_3d[2] = pt[2];

	tmp_corr.ptc.x = cc[0];
	tmp_corr.ptc.y = cc[1];
	tmp_corr.ptc.z = cc[2];
 
	pt_corr[i] = tmp_corr;
      }
    
    std::cout << ncam << " old cameras\n";

    return true;
}
