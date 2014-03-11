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


