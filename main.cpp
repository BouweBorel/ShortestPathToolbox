#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <utils/Point.h>
#include "obstacles/Obstacles.h"
#include "environment/Environment.h"
#include "visualization/Visualizer.h"




int main() {

    Environment environment;

    // Experimental room test environment
    std::vector<Point> envPoints = {
        {0,0},
        {200,0},
        {200,200},
        {0,200}
    };
    for (const Point& point : envPoints) {
        environment.addPoint(point);
    }

    Obstacles obstacles;
    

    // Experimental Room test obstacles
    std::vector<std::vector<Point>> roomDrawingObstacles1 = {
        {{35, 1}, {50, 1}, {50, 60}, {35, 60}},

        {{35, 100}, {50, 100}, {50, 199}, {35, 199}},

        {{115, 1}, {125, 1}, {125, 80}, {115, 80}},

        {{115, 120}, {125, 120}, {125, 199}, {115, 199}},

        {{150, 8}, {190, 8}, {190, 42}, {186, 42}, {186, 12}, {154, 12}, {154, 42}, {150, 42}}
    };

    // Experimental Room test raycasting obstacles
    std::vector<std::vector<Point>> roomObstacles1 = {
        {{30, 1}, {55, 1}, {55, 65}, {30, 65}},
        
        {{30, 95}, {55, 95}, {55, 199}, {30, 199}},
        
        {{110, 1}, {130, 1}, {130, 85}, {110, 85}},
        
        {{110, 115}, {130, 115}, {130, 199}, {110, 199}},

        {{145, 3}, {195, 3}, {195, 47}, {191, 47}, {191, 7}, {149, 7}, {149, 47}, {145, 47}}
    };

    // Experimental Room test scaled obstacles
    std::vector<std::vector<Point>> roomScaledObstacles1 = {
        {{27, 1}, {58, 1}, {58, 68}, {27, 68}},
        {{27, 92}, {58, 92}, {58, 199}, {27, 199}},
        {{107, 1}, {133, 1}, {133, 88}, {107, 88}},
        {{107, 112}, {133, 112}, {133, 199}, {107, 199}},
        {{142, 1}, {198, 1}, {198, 50}, {194, 50}, {194, 4}, {146, 4}, {146, 50}, {142, 50}}  
    };

    for (int i = 0; i < roomObstacles1.size(); i++) {
        Obstacle obstacle;
        obstacle.setCorners(roomObstacles1[i]); 
        obstacle.setScaledCorners(roomScaledObstacles1[i]);
        obstacle.setDrawnCorners(roomDrawingObstacles1[i]); 
        obstacles.addObstacle(obstacle);
    }
    obstacles.displayAll();

    Visualizer visualizer(800, 800);//(1200, 1200); // Window size of 1100x1100 pixels
    visualizer.run(obstacles, environment);
    
    return 0;
}