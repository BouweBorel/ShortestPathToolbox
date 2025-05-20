#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <vector>
#include "Obstacle.h"

class Obstacles {
public:
    void addObstacle(const Obstacle& obstacle);

    void displayAll() const;

    const std::vector<Obstacle>& getObstacles() const;

private:
    std::vector<Obstacle> obstacles;
};

#endif 