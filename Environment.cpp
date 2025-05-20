#include "Environment.h"
#include <iostream>
#include <CGAL/intersections.h>
#include <optional>


void Environment::addPoint(const Point& point) {
    points.push_back(point);
}

const std::vector<Point>& Environment::getPoints() const {
    return points;
}

bool Environment::isPointInside(const Point& point) const {
    bool inside = false;
    size_t n = points.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((points[i].y > point.y) != (points[j].y > point.y)) &&
            (point.x < (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool Environment::isSquareInside(const Point& center, float size) const {
    float halfSize = size / 2.0;
    std::vector<Point> corners = {
        {center.x - halfSize, center.y - halfSize},
        {center.x + halfSize, center.y - halfSize}, 
        {center.x + halfSize, center.y + halfSize}, 
        {center.x - halfSize, center.y + halfSize}  
    };

    for (const auto& corner : corners) {
        if (!isPointInside(corner)) {
            return false;
        }
    }
    return true;
}


void Environment::display() const {
    std::cout << "Environment Properties:\n";
    std::cout << "  Points:\n";
    for (const auto& point : points) {
        std::cout << "    (" << point.x << ", " << point.y << ")\n";
    }
}
