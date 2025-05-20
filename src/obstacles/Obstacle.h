#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include "../utils/Point.h"

class Obstacle {
public:
    Obstacle();

    void setCenter(float x, float y);

    void setSize(float size);

    void setCorners(const std::vector<Point>& corners);
    
    std::vector<Point> getObstacleCorners() const;

    void setScaledCorners(const std::vector<Point>& corners);

    std::vector<Point> getScaledObstacleCorners() const;

    void setDrawnCorners(const std::vector<Point>& corners);

    std::vector<Point> getDrawnCorners() const;

    Point getCenter() const;

    float getSize() const;

    std::vector<Point> getCorners() const;

    std::vector<Point> getRandomCorners() const;

    std::vector<Point> getInnerConvexPolygon() const;

    std::vector<Point> getShrunkenCorners() const;

    bool overlapsWith(const Obstacle& other) const;
    
    void display() const;

private:
    Point center;
    float size;  
    std::vector<Point> corners; 
    std::vector<Point> scaledCorners; 
    std::vector<Point> drawnCorners; 
};

#endif 