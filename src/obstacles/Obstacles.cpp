#include "Obstacles.h"
#include <iostream>

void Obstacles::addObstacle(const Obstacle& obstacle) {
    obstacles.push_back(obstacle);
}

void Obstacles::displayAll() const {
    for (size_t i = 0; i < obstacles.size(); ++i) {
        std::cout << "Obstacle " << i + 1 << ":\n";
        obstacles[i].display();
        std::cout << "\n";
    }
}

const std::vector<Obstacle>& Obstacles::getObstacles() const {
    return obstacles;
}