#include "Obstacle.h"
#include <iostream>
#include <cmath>
#include <algorithm>

Obstacle::Obstacle() : center{0.0, 0.0}, size{1.0}, corners{{0.0, 0.0}}, scaledCorners{{0,0}} {}

void Obstacle::setCenter(float x, float y) {
    center.x = x;
    center.y = y;
}

void Obstacle::setSize(float size) {
    if (size > 0) {
        this->size = size;
    } else {
        std::cerr << "Error: Size must be positive.\n";
    }
}

void Obstacle::setCorners(const std::vector<Point>& corners) {
    this->corners = corners;
}

std::vector<Point> Obstacle::getObstacleCorners() const{
    return corners;
}

void Obstacle::setScaledCorners(const std::vector<Point>& scaledCorners) {
    this->scaledCorners = scaledCorners;
}

std::vector<Point> Obstacle::getScaledObstacleCorners() const{
    return scaledCorners;
}

void Obstacle::setDrawnCorners(const std::vector<Point>& corners) {
    this->drawnCorners = corners;
}

std::vector<Point> Obstacle::getDrawnCorners() const {
    return drawnCorners;
}

Point Obstacle::getCenter() const {
    return center;
}

float Obstacle::getSize() const {
    return size;
}

std::vector<Point> Obstacle::getCorners() const {
    float halfSize = size / 2.0;
    return {
        {center.x - halfSize, center.y - halfSize}, 
        {center.x + halfSize, center.y - halfSize},
        {center.x + halfSize, center.y + halfSize},
        {center.x - halfSize, center.y + halfSize}
    };
}

std::vector<Point> Obstacle::getRandomCorners() const {
    
    std::vector<Point> corners = {
        Point(422.4, 108.2),
        Point(452.4, 108.2),
        Point(452.4, 138.2),
        Point(432.4, 125.2),
        Point(422.4, 138.2)
    };
    return corners;
}

std::vector<Point> Obstacle::getInnerConvexPolygon() const {
    std::vector<Point> corners = getRandomCorners();
    std::vector<Point> convexPolygon = corners;

    if (convexPolygon.size() < 4) {
        return convexPolygon;
    }

    auto cross = [](const Point& a, const Point& b, const Point& c) -> double {
        double dx1 = b.x - a.x;
        double dy1 = b.y - a.y;
        double dx2 = c.x - b.x;
        double dy2 = c.y - b.y;
        return dx1 * dy2 - dy1 * dx2;
    };

    int n = convexPolygon.size();
    bool foundSign = false;
    bool ccw = true;

    for (int i = 0; i < n; ++i) {
        const Point& a = convexPolygon[i];
        const Point& b = convexPolygon[(i + 1) % n];
        const Point& c = convexPolygon[(i + 2) % n];
        double cp = cross(a, b, c);
        if (std::abs(cp) > 1e-8) {
            ccw = (cp > 0);
            foundSign = true;
            break;
        }
    }

    if (!foundSign) {
        return {};
    }

    for (int i = 0; i < (int)convexPolygon.size(); ) {
        int n = convexPolygon.size();
        const Point& prev = convexPolygon[(i - 1 + n) % n];
        const Point& curr = convexPolygon[i];
        const Point& next = convexPolygon[(i + 1) % n];

        double cp = cross(prev, curr, next);
        bool isConvex = ccw ? (cp > -1e-8) : (cp < 1e-8); 

        if (!isConvex) {
            convexPolygon.erase(convexPolygon.begin() + i);
        } else {
            ++i;
        }

        if (convexPolygon.size() <= 3) break;
    }

    return convexPolygon;
}

std::vector<Point> Obstacle::getShrunkenCorners() const{
    float halfShrunkenSize = (size+1.0) / 2.0;
    return {
        {center.x - halfShrunkenSize, center.y - halfShrunkenSize}, 
        {center.x + halfShrunkenSize, center.y - halfShrunkenSize}, 
        {center.x + halfShrunkenSize, center.y + halfShrunkenSize},
        {center.x - halfShrunkenSize, center.y + halfShrunkenSize} 
    };
}

bool Obstacle::overlapsWith(const Obstacle& other) const {
    float halfSize1 = size / 2.0;
    float halfSize2 = other.size / 2.0;

    bool overlapX = std::abs(center.x - other.center.x) <= (halfSize1 + halfSize2);

    bool overlapY = std::abs(center.y - other.center.y) <= (halfSize1 + halfSize2);

    return overlapX && overlapY;
}

void Obstacle::display() const {
    std::cout << "Obstacle Properties:\n";
    std::cout << "  Center: (" << center.x << ", " << center.y << ")\n";
    std::cout << "  Size: " << size << "\n";
    std::cout << "  Corners:\n";
    auto localCorners = getObstacleCorners();
    for (const auto& corner : localCorners) {
        std::cout << "    (" << corner.x << ", " << corner.y << ")\n";
    }
}