#include "Visualizer.h"
#include <iostream>



void Visualizer::drawShortestPath(const std::vector<Point>& path,
    float scaleX, float scaleY,
    const sf::Color& color,
    float thickness) 
{
    if (path.empty()) return;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(path[i].x * scaleX, path[i].y * scaleY), color),
            sf::Vertex(sf::Vector2f(path[i+1].x * scaleX, path[i+1].y * scaleY), color)
        };
        window.draw(line, 2, sf::Lines);
    }

    for (const auto& point : path) {
        sf::CircleShape node(thickness);
        node.setPosition(point.x * scaleX - thickness, point.y * scaleY - thickness);
        node.setFillColor(color);
        window.draw(node);
    }
}


void Visualizer::drawVisibilityPolygon(
    const Point& source,
    std::vector<Point>& visiblePoints,
    float scaleX,
    float scaleY,
    const sf::Color& outlineColor,
    float thickness
) {
    sf::ConvexShape visibilityPolygon;
    visibilityPolygon.setPointCount(visiblePoints.size());

    for (size_t i = 0; i < visiblePoints.size(); ++i) {
        float x = visiblePoints[i].x * scaleX;
        float y = visiblePoints[i].y * scaleY;
        visibilityPolygon.setPoint(i, sf::Vector2f(x, y));
    }

    visibilityPolygon.setFillColor(sf::Color::Transparent);
    visibilityPolygon.setOutlineColor(outlineColor);
    visibilityPolygon.setOutlineThickness(thickness);

    window.draw(visibilityPolygon);
}

Visualizer::Visualizer(int width, int height)
    : window(sf::VideoMode(width, height), "Obstacle Visualizer") {}

    
sf::Vector2f Visualizer::toSFMLCoordinates(const Point& point, float scaleX, float scaleY) const {
    return sf::Vector2f(point.x * scaleX, point.y * scaleY);
}

void Visualizer::drawLine(const Point& p1, const Point& p2, const sf::Color& color, float scaleX, float scaleY) {
    sf::Vertex line[] = {
        sf::Vertex(toSFMLCoordinates(p1, scaleX, scaleY), color),
        sf::Vertex(toSFMLCoordinates(p2, scaleX, scaleY), color)
    };
    window.draw(line, 3, sf::Lines);
}

void Visualizer::drawPath(
    const std::vector<Point>& path, 
    float scaleX, float scaleY, 
    const sf::Color& color
) {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        drawLine(path[i], path[i + 1], color, scaleX, scaleY);
    }
}

void Visualizer::run(const Obstacles& obstacles, const Environment& environment) {
    float scaleX = window.getSize().x / 200;//700.0; // 700 is the maximum coordinate
    float scaleY = window.getSize().y / 200;//700.0;

    std::vector<Point> drawingPoints;
    for (const auto& obstacle : obstacles.getObstacles()) {
        auto corners = obstacle.getObstacleCorners();
        drawingPoints.insert(drawingPoints.end(), corners.begin(), corners.end());
    }

    std::vector<Point> searchPoints;
    
    for (const auto& obstacle : obstacles.getObstacles()) {
        auto corners = obstacle.getScaledObstacleCorners();
        searchPoints.insert(searchPoints.end(), corners.begin(), corners.end());
    }

    Point start = {20, 20};//{120, 200};//{500, 300};//{430, 80}; // Example start point
    Point end = {170, 30};//{900, 1000};//{800, 800};//{520, 650}; // Example end point

    RayCasting rayCaster(environment, obstacles);
    PathFinder pathfinder;
   
    auto adjMatrix = rayCaster.computeAdjacencyMatrix(searchPoints, nullptr);
    
    rayCaster.printFloatMatrix(adjMatrix);

    std::vector<std::pair<int, Polygon_2>> visibilityPolygons;
    
    bool needsRedraw = true;
    while (window.isOpen()) {
        sf::Event event;
        window.waitEvent(event); 

        if (event.type == sf::Event::Closed)
            window.close();
        else if (event.type == sf::Event::Resized)
            needsRedraw = true;

        if (needsRedraw) {
            needsRedraw = false;
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);

        sf::ConvexShape environmentPolygon;
        environmentPolygon.setPointCount(environment.getPoints().size());
        for (size_t i = 0; i < environment.getPoints().size(); ++i) {
            float x = environment.getPoints()[i].x * scaleX;
            float y = environment.getPoints()[i].y * scaleY;
            environmentPolygon.setPoint(i, sf::Vector2f(x, y));
        }
        environmentPolygon.setFillColor(sf::Color::Transparent);
        environmentPolygon.setOutlineColor(sf::Color::Blue);
        environmentPolygon.setOutlineThickness(5);
        window.draw(environmentPolygon);

        int index = 0;
        for (const auto& obstacle : obstacles.getObstacles()) {
            auto corners = obstacle.getDrawnCorners();

            if (! (corners[0] == corners[index] && index == 3)) {
                corners.push_back(corners[0]);
            }
            sf::ConvexShape obstacleShape;
            obstacleShape.setPointCount(corners.size());
            for (size_t i = 0; i < corners.size(); ++i) {
                float x = corners[i].x * scaleX;
                float y = corners[i].y * scaleY;
                obstacleShape.setPoint(i, sf::Vector2f(x, y));
            }
            obstacleShape.setFillColor(sf::Color::Red);
            obstacleShape.setOutlineColor(sf::Color::Black);
            obstacleShape.setOutlineThickness(2);

            window.draw(obstacleShape);

        }

        for (int i = 0; i < searchPoints.size() ; i++) {
            auto visPolygon_cgal = rayCaster.computeVisibilityPolygon2(index, searchPoints[i]);
            visibilityPolygons.push_back(visPolygon_cgal);
            std::vector<Point> visiblePoints = rayCaster.convertToVectorPoints(visPolygon_cgal.second);

            if (0){
                sf::ConvexShape visibilityPolygon;
                visibilityPolygon.setPointCount(visiblePoints.size());
                for (size_t i = 0; i < visiblePoints.size(); ++i) {
                    float x = visiblePoints[i].x * scaleX;
                    float y = visiblePoints[i].y * scaleY;
                    visibilityPolygon.setPoint(i, sf::Vector2f(x, y));
                }
                visibilityPolygon.setFillColor(sf::Color::Transparent);
                visibilityPolygon.setOutlineColor(sf::Color::Yellow);
                visibilityPolygon.setOutlineThickness(4);
                window.draw(visibilityPolygon);
            }
            index++;
        }

        auto subregions = rayCaster.windowSubdivision(200,200,5);
        
        for (const auto& subregion : subregions) {
            sf::ConvexShape subregionShape;
            std::vector<Point> sRCorners = rayCaster.convertToVectorPoints(subregion);
            if (!sRCorners.empty()) {
                std::cout << "Test001\n";
                subregionShape.setPointCount(sRCorners.size());
                for (size_t i = 0; i < sRCorners.size(); ++i) {
                    float x = sRCorners[i].x * scaleX;
                    float y = sRCorners[i].y * scaleY;
                    std::cout << "x: " << x << ", y: " << y << "\n";
                    subregionShape.setPoint(i, sf::Vector2f(x, y));
                }
                subregionShape.setFillColor(sf::Color(255, 0, 0, 50)); // Semi-transparent red
                subregionShape.setOutlineColor(sf::Color::Black);
                subregionShape.setOutlineThickness(1);

                window.draw(subregionShape);
            }
        }
    
        printf("Number of subregions : %ld\n", subregions.size());
        std::cout << "Pass the test 1! \n";

        std::cout << "Number of visibility Polygons in visualizer: " << visibilityPolygons.size() << "\n";
        auto subregionsMap = rayCaster.computeSubregionsSearch(subregions, visibilityPolygons);
        
        for (const auto& [region, polygons] :subregionsMap) {
            std::cout << "Subregion: " << region << "\n";
            std::cout << "Number of polygons in subregion: " << polygons.size() << "\n";
            for (const auto& polygon : polygons) {
                std::cout << "Polygon ID: " << polygon.first << "\n";
            }
        }

        std::vector<std::vector<float>> extendedAdjacencyMatrix = rayCaster.extendingAdjacencyMatrix(
            Point_2(start.x, start.y),
            Point_2(end.x, end.y),
            adjMatrix,
            subregions,
            searchPoints,
            visibilityPolygons
        );

        std::cout << "Extended Matrix: \n";
        rayCaster.printFloatMatrix(extendedAdjacencyMatrix);
        
        searchPoints.push_back(start);
        searchPoints.push_back(end);

        auto path = pathfinder.boostShortestPath(extendedAdjacencyMatrix, searchPoints, start, end);
        
        sf::CircleShape startPoint(5.0f);
        startPoint.setPosition(start.x * scaleX - 5, start.y * scaleY - 5);
        startPoint.setFillColor(sf::Color::Green);
        window.draw(startPoint);
        sf::CircleShape endPoint(5.0f);
        endPoint.setPosition(end.x * scaleX - 5, end.y * scaleY - 5);
        endPoint.setFillColor(sf::Color::Blue);
        window.draw(endPoint);

        std::cout << "Shortest path coordinates: \n";
        int count = 0;
        for (const auto& point : path) {
            std::cout << "P" << count << " [" << point.x << ", " << point.y << "]\n"; 
            count++;
        }
        count =0;
        
        if (!path.empty()) {
            drawPath(path, scaleX, scaleY, sf::Color::Green);
        }
        window.display();
    }
}
}

