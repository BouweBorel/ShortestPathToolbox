# ShortestPathToolbox
A toolbox which generate the shortest path between two points in a given environment

## Usage manual
To use this toolbox, you need to input the following:
- **Environment coordinates**: It is the points that forme the polygon enclosing your obstacles
      * Need to be feeded into the vector of points "envPoints" that is in the "main" file
  
  ![image](https://github.com/user-attachments/assets/f384d0ff-cd6e-4943-94b7-b062e8c13c54)

- **Obstacle coordinates**: It is the points forming the obstacle in your environment. It has to be feeded in 3 different form
    - **Drawing Obstacle**: Which is the coordinates of the actual your obstacles as they are in your referential. Each lines you see represent an obstacle. 

      ![image](https://github.com/user-attachments/assets/e812f1ff-a537-41c6-87b7-a2871a94f00e)

      For complex polygons, it is recommended to be decomposed in set of rectangles or triangle. For example this polygon (bounded with black lines) needed to be feeded as 3 rectangle instead of 1 to avoid this error
        ![Screenshot from 2025-05-07 14-00-00](https://github.com/user-attachments/assets/0cfb9d14-2979-483d-bd14-4bc8280d7fc0)

    - **Scaled Obstacle**: It is the same coordinates than **Drawing Obstacles** but it is scale by a distance , $d$ , you will like the path to be away from the obstacle. Complex polygons, can be left as they are.

      ![image](https://github.com/user-attachments/assets/d36ec546-d278-4059-a709-5ae052b24939)


      To get those points, use the following prompt into an LLM if you have complex obstacles:

          Scale these polygons outward by $d$ pixels:
          std::vector<std::vector<Point>> roomDrawingObstacles1 = {
            //Enter all your drawing obstacles here
            {{35, 1}, {50, 1}, {50, 60}, {35, 60}},
            {{35, 100}, {50, 100}, {50, 199}, {35, 199}},
            {{115, 1}, {125, 1}, {125, 80}, {115, 80}},
            {{115, 120}, {125, 120}, {125, 199}, {115, 199}},
            {{150, 8}, {190, 8}, {190, 42}, {186, 42}, {186, 12}, {154, 12}, {154, 42}, {150, 42}} (example of a complex polygon)
          };
    
    - **Obstacle** : Same as **Scaled Obstacles** but scaled by $d/2$
      
      ![image](https://github.com/user-attachments/assets/775d9ac1-a6d8-4174-8ca1-b093713a6bdb)

- **Size of the window**: It is the size of the window, it can be set here

    ![image](https://github.com/user-attachments/assets/1b86cd26-f378-4ac6-bca8-5869d440dd9d)

- **Size of region domain and subdivision**: The first to arguments are the size of region domain and the last is the number of subdivision (n x n) size

    ![image](https://github.com/user-attachments/assets/ba503b88-4f38-416b-865e-a33a653b780c)

    Example of a 4 x 4 grid:
  
    ![Screenshot from 2025-05-07 12-39-03](https://github.com/user-attachments/assets/37b94725-e1cc-482f-a5ea-762cb751a83f)


- **Scaling factor**: It is the ratio between the window size and the region domain (n x n, which covers your environment) size. You just need to substitute this denominator with yours

    ![image](https://github.com/user-attachments/assets/2cf51207-7795-4222-85d6-58ec9a0e4630)



## Installation guide
This project was build on ubuntu, in other to run this project follow these steps (all this can be done into a terminal, wr): 

1- After cloning this repository in a file, change your directory to the "external folder" cloned

2- Inside this "external folder", the proceed to the installation of the prerequire libraries:

  2.1 - Download CGAL library from this website ant extract:
        https://github.com/CGAL/cgal/releases/download/v5.6/CGAL-5.6-library.tar.xz
        
        tar -xf CGAL-5.6-library.tar.xz
        
  2.2 - Download Boost library from this website ant extractBoost:
        https://boostorg.jfrog.io/artifactory/main/release/1.87.0/source/boost_1_87_0.tar.gz
        
        tar -xzf boost_1_87_0.tar.gz
        
  2.3 - Download GMP library from this website ant extract:
        https://gmplib.org/download/gmp/gmp-6.3.0.tar.xz
        
        tar -xf gmp-6.3.0.tar.xz
        
  2.4 - Download MPFR from this website ant extract:
        https://www.mpfr.org/mpfr-current/mpfr-4.2.1.tar.xz
        
        tar -xf mpfr-4.2.1.tar.xz

  2.5- Install SFML library using this command line in the terminal: 
  
        sudo apt install -y libsfml-dev
        
3- Move out from "external folder" and create a folder called "build"

4- Move inside the "build" folder, then use the following commands:

        cmake -S ..
        make
        
5- Once the project is build into the "build" folder, run the project using this command:

        ./SPToolbox


## FAQ
If there is any issue, submit a git issue and you will be answered.

Thanks for consulting
