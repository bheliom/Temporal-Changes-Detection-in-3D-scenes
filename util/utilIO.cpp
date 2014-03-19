#include "utilIO.hpp"

void readCmdInput(std::map<int,std::string> &inStrings, int argc, char** argv){
  
  int flags, opt;
  int nsecs, tfnd;

  nsecs = 0;
  tfnd = 0;
  flags = 0;

  while ((opt = getopt(argc, argv, "m:p:b:i:o:")) != -1) {
    switch (opt) {
   
    case 'm':
      inStrings[MESH] = optarg;
      break;
    case 'p':
      inStrings[PMVS] = optarg;
      break;
    case 'b':
      inStrings[BUNDLER] = optarg;
      break;
    case 'i':
      inStrings[IMAGELIST] = optarg;
      break;
    case 'o':
      inStrings[OUTDIR] = optarg;
      break;
    
    default: /* '?' */
      fprintf(stderr, "Usage: %s [-m input mesh] [-p input PMVS] [-b input bundler file] [-i input image list]\n",
	      argv[0]);
    }
  }
}

void inputHandler(std::vector<std::string> inputStrings){

}

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
  std::cout<<"jestem tu";
  if(importVar.Open(m, shots , image_filenames, filename.c_str(), filename_images.c_str()))
     std::cout<<"Error reading the bundler files!"<<std::endl;

}










