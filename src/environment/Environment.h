#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include "obstacles/Obstacles.h"
#include "utils/Point.h"


class Environment {
public:
    void addPoint(const Point& point);

    const std::vector<Point>& getPoints() const;

    bool isPointInside(const Point& point) const;

    bool isSquareInside(const Point& center, float size) const;

    void display() const;

private:
    std::vector<Point> points; 
};

#endif // ENVIRONMENT_H