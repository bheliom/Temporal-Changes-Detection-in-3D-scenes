#include "utilIO.hpp"

void callVsfm(std::string dataPath, std::string outputPath){
  
  std::string outCommand;
  std::string vsfmCommand("VisualSFM sfm+pmvs ");
  outCommand = vsfmCommand+" "+dataPath+" "+outputPath;
  
  system(outCommand.c_str());
}

void getPlyFileVcg(std::string filename, MyMesh &m){

  vcg::tri::io::ImporterPLY<MyMesh> importVar;
  
  if(importVar.Open(m,filename.c_str()))
    {
      printf("Error reading file  %s\n",filename.c_str());
      std::cout<<vcg::tri::io::ImporterOFF<MyMesh>::ErrorMsg(importVar.Open(m,filename.c_str()))<<std::endl;
      exit(0);
    }
  std::cout<<"Mesh loaded correctly. No. of faces:"<<m.FN()<<" no. of vertices:"<<m.VN()<<std::endl;
}

void savePlyFileVcg(std::string filename, MyMesh &m){
  
  vcg::tri::io::ExporterPLY<MyMesh> exportVar;

  exportVar.Save(m,filename.c_str(),vcg::tri::io::Mask::IOM_VERTCOLOR);

}

void getBundlerFile(MyMesh &m, std::string filename, std::string filename_images, std::vector<vcg::Shot<float> > &shots, std::vector<std::string> &image_filenames){

  vcg::tri::io::ImporterOUT<MyMesh> importVar;

  if(importVar.Open(m, shots , image_filenames, filename.c_str(), filename_images.c_str()))
     std::cout<<"Error reading the bundler files!"<<std::endl;

}










