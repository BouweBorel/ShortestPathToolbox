cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(ShortestPathToolbox VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find SFML (required for visualization)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Find SFML (required for visualization)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Manually specify paths to CGAL and its dependencies
set(CGAL_DIR "${CMAKE_SOURCE_DIR}/external/cgal")
set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/external/boost_1_87_0")
set(GMP_DIR "${CMAKE_SOURCE_DIR}/external/gmp-6.3.0")
set(MPFR_DIR "${CMAKE_SOURCE_DIR}/external/mpfr-4.2.1")

# Include CGAL
find_package(CGAL REQUIRED)

# Include directories
include_directories(src)
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${GMP_INCLUDE_DIRS})
include_directories(${MPFR_INCLUDE_DIRS})


# Add the main executable
add_executable(SPToolbox
    src/main.cpp
    src/obstacles/Obstacle.cpp
    src/obstacles/Obstacles.cpp
    src/environment/Environment.cpp
    src/visualization/Visualizer.cpp
    src/algorithms/RayCasting.cpp
    src/algorithms/DelaunayTriangulation.cpp  
    src/algorithms/PathFinder.cpp  
    src/algorithms/PolyVisibility.cpp
)
#   src/environment/Environment.cpp
#    src/obstacle/Obstacle.cpp
#   src/visualization/Visualizer.cpp

# Link all libraries to the main executable
target_link_libraries(SPToolbox 
                        sfml-graphics 
                        sfml-window 
                        sfml-system 
                        sfml-audio 
                        ${CGAL_LIBRARIES} 
                        ${GMP_LIBRARIES} 
                        ${MPFR_LIBRARIES}
                    )

# Add a target for unit tests (optional)
#enable_testing()
#add_executable(Tests
#    tests/test_environment.cpp
#    tests/test_obstacle.cpp
#    src/environment/Environment.cpp
#    src/obstacle/Obstacle.cpp
#)
#target_link_libraries(Tests sfml-graphics sfml-window sfml-system)
#add_test(NAME Tests COMMAND Tests)
