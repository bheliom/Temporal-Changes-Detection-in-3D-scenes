UPDATE(25.08.2014): FINAL VERSION OF THE SOFTWARE WITH INSTRUCTIONS WILL BE REALEASED SOON.

============
Temporal changes detection in scenes reconstructed using VisualSfM
============

Source code of the project developed for the master thesis "5D scene reconstruction using crowdsourcing cameras".

Author: Waldemar Franczak.
Year: 2014

Dependencies:
- VisualSFM(command line functionality is required )
- VCG lib
- OpenCV v.2.4.8
- PCL 1.3 lib
- boost lib
- maxflow-v3.01 (maxflow/mincut lib from http://vision.csd.uwo.ca/code/)

Important info:
- VisualSfM command line functionality requires CUDA enabled hardware if SIFT features and image matches have to be computed. In order to use the code on computers without CUDA, SIFT and matches have to be precomputed for new set of images before calling the change detector.

- in CMakeLists on line 13 there is a fixed directory to VCG wrap module /usr/include/wrap/ply/plylib.cpp which has to be changed if your dirrectory of plylib.cpp is different. Also you can just put compiled plylib.cpp file in the build directory.

- in order to operate on NVM files from VisualSfM, project makes use of modified code(project files util/pbaUtil.*) from "Multicore Bundle Adjustment" project by Changchang Wu http://grail.cs.washington.edu/projects/mcba/

- VCG library file import_out.h which is used in GUI of MeshLab and is dependent on Qt framework had to be modified in order to be able to read bundler files without Qt. Due to licensing this modified file is not included however main algorithms can be run without reading bundler files.

The code was developed and tested on Ubuntu 12.04, 64bit operating system.

EXAMPLE USE:

In order to use the code after successful compilation we need:
- old_nvm_file - NVM file from VisualSfM of an old model
- list_of_new_images - text file containing set of directories to new images(1 dir. per image)
- 3d_model - PLY file obtained from reconstruction of old model
- output_nvm_name - name of NVM file that will be created after calling VisualSfM(arbitrary name)

From command line type:
./TempChangeDetect -b old_nvm_file.nvm -p list_of_new_images.txt -o output_nvm_name.nvm -m 3d_model.ply