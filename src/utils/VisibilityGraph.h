#ifndef VISIBILTYGRAPH_H
#define VISIBILITYGRAPH_H

#include <vector>
#include <unordered_map>
#include <cmath>
#include "Point.h"

struct VisibilityGraph {
    std::unordered_map<Point, std::vector<std::pair<Point, double>>> adjacency_list;
    
    void addEdge(const Point& from, const Point& to) {
        double dx = from.x - to.x;
        double dy = from.y - to.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        adjacency_list[from].emplace_back(to, distance);
        adjacency_list[to].emplace_back(from, distance); // Undirected graph
    }
};


#endif