# ShortestPathToolbox
A toolbox which generate the shortest path between two points in a given environment


## Prerequisites
This project was build on ubuntu, in other to run this project follow these steps (all this can be done into a terminal, wr): 

1- After cloning this repository in a file, change your directory to the "external folder" cloned

2- Inside this "external folder", the proceed to the installation of the prerequire libraries:

  2.1 - Download and extract CGAL:
        https://github.com/CGAL/cgal/releases/download/v5.6/CGAL-5.6-library.tar.xz
        tar -xf CGAL-5.6-library.tar.xz
        mv CGAL-5.6-library cgal
        
  2.2 - Download and extract Boost (example - adjust version as needed):
        https://boostorg.jfrog.io/artifactory/main/release/1.87.0/source/boost_1_87_0.tar.gz
        tar -xzf boost_1_87_0.tar.gz
        
  2.3 - Download and extract GMP:
        https://gmplib.org/download/gmp/gmp-6.3.0.tar.xz
        tar -xf gmp-6.3.0.tar.xz
        
  2.4 - Download and extract MPFR:
        https://www.mpfr.org/mpfr-current/mpfr-4.2.1.tar.xz
        tar -xf mpfr-4.2.1.tar.xz
        
3- Move out from "external folder" and create a folder called "build"

4- Move inside the "build" folder, then use the following 


cd ..

./SPToolbox
