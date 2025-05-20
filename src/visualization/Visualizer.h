#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <SFML/Graphics.hpp>
#include "../obstacles/Obstacles.h"
#include "../environment/Environment.h"
#include "../algorithms/RayCasting.h"
#include "../algorithms/PathFinder.h" 


class Visualizer {
public:
    Visualizer(int width, int height);

    void run(const Obstacles& obstacles, const Environment& environment);

    void drawShortestPath(const std::vector<Point>& path, 
        float scaleX, float scaleY,
        const sf::Color& color = sf::Color::Cyan,
        float thickness = 3.0f);

    void drawVisibilityPolygon(
        const Point& source,
        std::vector<Point>& visiblePoints,
        float scaleX,
        float scaleY,
        const sf::Color& outlineColor,
        float thickness
    );

private:
    sf::RenderWindow window;
    PathFinder pathfinder; 
    
    std::vector<std::pair<Point, Point>> computeDelaunayTriangulation(
        const std::vector<Point>& sources,
        const std::vector<Point>& targetPoints,
        const Environment& environment,
        const Obstacles& obstacles
    );

    sf::Vector2f toSFMLCoordinates(const Point& point, float scaleX, float scaleY) const;

    void drawLine(const Point& p1, const Point& p2, const sf::Color& color, float scaleX, float scaleY);

    void drawPath(const std::vector<Point>& path, float scaleX, float scaleY, const sf::Color& color = sf::Color::Green);
};

#endif 