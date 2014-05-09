finalProject
============

Source code of the project developed for the master thesis project "5D scene reconstruction using crowdsourcing cameras".

Author: Waldemar Franczak

Dependencies:

VCG lib
OpenCV v.2.4.8
PCL lib
boost lib

Additional info:

- in order to operate on NVM files from VisualSfM, project makes use of modified code(pbaUtil.*) from "Multicore Bundle Adjustment" project by Changchang Wu http://grail.cs.washington.edu/projects/mcba/

-VCG library file import_out.h which is used in GUI of MeshLab and is dependent on Qt framework had to be modified in order to be able to read bundler files without Qt. Due to licensing this modified file is not included however main algorithms can be run without reading bundler files.
