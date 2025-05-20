#ifndef POINT_H
#define POINT_H

#include <functional>

struct Point {
    float x;
    float y;
    Point(float x, float y) : x(x), y(y) {}

    // Define the < operator for Point
    bool operator<(const Point& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
    // Equality operator for unordered_map
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

// Custom hash function for Point
namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            // Combine the hash of x and y using a simple method
            return hash<float>()(p.x) ^ (hash<float>()(p.y) << 1);
        }
    };
}

#endif // POINT_H