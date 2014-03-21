#ifndef __UTILIO_H_INCLUDED__
#define __UTILIO_H_INCLUDED__

#include <string>
#include <unistd.h>
#include <map>

#include "../common/common.hpp"

#include<wrap/io_trimesh/import_off.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

void readCmdInput(std::map<int,std::string> &inputStrings, int argc, char** argv);

void inputHandler(std::vector<std::string> inputStrings);

void callVsfm(std::string dataPath, std::string outputPath);

void getPlyFileVcg(std::string filename, MyMesh &m);

void savePlyFileVcg(std::string filename, MyMesh &m);

void getBundlerFile(std::string filename);

void getBundlerFile(MyMesh &m, std::string filename, std::string filename_images, std::vector<vcg::Shot<float> > &shots, std::vector<std::string> &image_filenames);

template <typename T>
void getPlyFilePCL(std::string filename, boost::shared_ptr<pcl::PointCloud<T> > outCloud){

if(pcl::io::loadPLYFile<T> (filename, *outCloud) == -1)
  PCL_ERROR ("Couldn't read file\n"); 
}

/* Function taken from http://stackoverflow.com/questions/60221/how-to-animate-the-command-line */
inline void DrawProgressBar(int len, double percent) {
  std::cout << "\x1B[2K"; // Erase the entire current line.
  std::cout << "\x1B[0E"; // Move to the beginning of the current line.
  std::string progress;
  for (int i = 0; i < len; ++i) {
    if (i < static_cast<int>(len * percent)) {
      progress += "=";
    } else {
      progress += " ";
    }
  }
  std::cout << "[" << progress << "] " << (static_cast<int>(100 * percent)) << "%";
  std::flush(std::cout); // Required.
}

#endif
