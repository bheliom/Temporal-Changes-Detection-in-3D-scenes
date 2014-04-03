#include "pbaUtil.h"

using namespace std;
bool LoadNVM(ifstream& in, vector<CameraT>& camera_data, vector<string>& names)
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

    std::cout << ncam << " cameras\n";

    return true;
}
