#include "RayCasting.h"
#include <CGAL/intersections.h>
#include <CGAL/Polygon_2.h>

RayCasting::RayCasting(const Environment& environment, const Obstacles& obstacles)
    : environment(environment), obstacles(obstacles) {}

bool RayCasting::isPointInsideObstacle(const Point& point) const {
    for (const auto& obstacle : obstacles.getObstacles()) {
        auto corners = obstacle.getCorners();
        float minX = corners[0].x, maxX = corners[0].x;
        float minY = corners[0].y, maxY = corners[0].y;

        for (const auto& corner : corners) {
            if (corner.x < minX) minX = corner.x;
            if (corner.x > maxX) maxX = corner.x;
            if (corner.y < minY) minY = corner.y;
            if (corner.y > maxY) maxY = corner.y;
        }

        if (point.x > minX && point.x < maxX && point.y > minY && point.y < maxY) {
            return true;
        }
    }
    return false; 
}

bool RayCasting::isPointInsideCGALObstacle(const Point_2& point) const {
    
    for (const auto& obstacle : obstacles.getObstacles()) {
        auto corners = obstacle.getScaledObstacleCorners();
        
        Polygon_2 polygon;
        for (const auto& corner : corners) {
            polygon.push_back(Point_2(corner.x, corner.y));
        }
        
        if (!polygon.is_simple()) {
            continue; 
        }
        
        if(polygon.has_on_bounded_side(point)) {
            return true; 
        }
    }
    return false;
}

bool RayCasting::castRay(const Point& source, const Point& target, const Obstacle* excludeObstacle) const {
    if (isPointInsideCGALObstacle(Point_2(target.x, target.y))) {
        return false; 
    }
    
    Segment_2 segment(Point_2(source.x, source.y), Point_2(target.x, target.y));

    for (const auto& obstacle : obstacles.getObstacles()) {
        auto corners  = obstacle.getObstacleCorners();
        for (size_t i = 0; i < corners.size(); ++i) {
            size_t j = (i + 1) % corners.size();
            Segment_2 edge(Point_2(corners[i].x, corners[i].y), Point_2(corners[j].x, corners[j].y));
            
            if (CGAL::do_intersect(segment, edge)) { 
                auto result = CGAL::intersection(segment, edge);
                if (const Point_2* p = std::get_if<Point_2>(&*result)) {
                    if (*p != segment.source() && *p != segment.target()) {
                        return false;
                    }
                    
                }
            }
        }
    }
    
    for (size_t i = 0; i < environment.getPoints().size(); ++i) {
        size_t j = (i + 1) % environment.getPoints().size();
        Segment_2 edge(Point_2(environment.getPoints()[i].x, environment.getPoints()[i].y),
                          Point_2(environment.getPoints()[j].x, environment.getPoints()[j].y));

        if (CGAL::do_intersect(segment, edge)) {
            auto result = CGAL::intersection(segment, edge);
            if (const Point_2* p = std::get_if<Point_2>(&*result)) {
                if (*p != segment.source() && *p != segment.target()) {
                    return false;
                }
            }
        }
    }

    return true;
}

std::vector<Point> RayCasting::computeVisibility(const Point& source, const std::vector<Point>& targetPoints, const Obstacle* excludeObstacle) {
    std::vector<Point> visiblePoints;
    std::vector<std::vector<int>> adjacencyMatrix;  
    for (const auto& target : targetPoints) {
        std::vector<int> adjColomn;
        if (castRay(source, target, excludeObstacle)) {
            visiblePoints.push_back(target);
            adjColomn.push_back(1);
        } else {
            adjColomn.push_back(0);
        }

        adjacencyMatrix.push_back(adjColomn);
    }
    visibilityPoints[source] = visiblePoints; 
    return visiblePoints;
}

std::vector<std::vector<int>> RayCasting::getVisibilityMatrix(const std::vector<Point>& points) const {
    std::vector<std::vector<int>> matrix(points.size(), std::vector<int>(points.size(), 0));
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j) continue;
            if (castRay(points[i], points[j])) {
                matrix[i][j] = 1;
            }
        }
    }
    return matrix;
}

std::vector<int> RayCasting::getAdjacencyColumn(
    const Point& source, 
    const std::vector<Point>& targetPoints, 
    const Obstacle* excludeObstacle) const { 
    std::vector<int> adjColumn;
    for (const auto& target : targetPoints) {
        if (castRay(source, target, excludeObstacle)) {
            adjColumn.push_back(1);
        } else {
            adjColumn.push_back(0);
        }
    }
    return adjColumn;
}



std::vector<std::vector<float>> RayCasting::computeAdjacencyMatrix(
    const std::vector<Point>& points, 
    const Obstacle* excludeObstacle) const {
    std::vector<std::vector<float>> adjacencyMatrix(points.size(), std::vector<float>(points.size(), 0));
    
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) {
            if (i != j && castRay(points[i], points[j], excludeObstacle)) {
                float dx = points[j].x - points[i].x;
                float dy = points[j].y - points[i].y;
                float distance = std::sqrt(dx*dx + dy*dy);
                adjacencyMatrix[i][j] = distance;
            }
        }
    }
    
    return adjacencyMatrix;
}

void RayCasting::printAdjacencyColumn(const Point& origin, const std::vector<int>& column) {
    std::cout << "Adjacency column for point (" << origin.x << "," << origin.y << "):\n[";
    for (size_t i = 0; i < column.size(); ++i) {
        std::cout << column[i];
        if (i < column.size() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
}

std::vector<std::vector<Point*>> RayCasting::getVisiVectors(
    const Point& source, 
    const std::vector<Point>& targetPoints, 
    const Obstacle* excludeObstacle) const
{
    std::vector<std::vector<Point*>> visiVect;
    
    for (const auto& target : targetPoints) {
        std::vector<Point*> visiblePoints;
        
        if (castRay(source, target, excludeObstacle)) {
            visiblePoints.push_back(const_cast<Point*>(&target));
        }
        
        visiVect.push_back(visiblePoints);
    }
    
    return visiVect;
}

void RayCasting::printVisibilityMatrix(const std::vector<Point>& points) const {
    auto visibilityMatrix = getVisibilityMatrix(points);

    std::cout << "Visibility Adjacency Matrix:\n";
    std::cout << "    ";
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << "P" << i << " ";
    }
    std::cout << "\n";

    for (size_t i = 0; i < visibilityMatrix.size(); ++i) {
        std::cout << "P" << i << " ";
        for (size_t j = 0; j < visibilityMatrix[i].size(); ++j) {
            std::cout << " " << visibilityMatrix[i][j] << " ";
        }
        std::cout << "\n";
    }
}

void RayCasting::printFloatMatrix(const std::vector<std::vector<float>>& visibilityMatrix) const {
    std::cout << "Visibility Adjacency Matrix:\n";
    std::cout << "    ";
    for (size_t i = 0; i < visibilityMatrix.size(); ++i) {
        std::cout << "P" << i << " ";
    }
    std::cout << "\n";

    for (size_t i = 0; i < visibilityMatrix.size(); ++i) {
        std::cout << "P" << i << " ";
        for (size_t j = 0; j < visibilityMatrix[i].size(); ++j) {
            std::cout << " " << visibilityMatrix[i][j] << " ";
        }
        std::cout << "\n";
    }
}   


std::vector<Point> RayCasting::getVisiblePoints(const Point& source) const {
    if (visibilityPoints.find(source) != visibilityPoints.end()) {
        return visibilityPoints.at(source);
    }
    return {};
}

std::vector<Point> RayCasting::getVisiblePointsByType(const Point& source, const std::string& type) const {
    return {};
}

Polygon_2 RayCasting::convertToCGALPolygon(const std::vector<Point>& points) const {
    Polygon_2 polygon;
    for (const auto& point : points) {
        polygon.push_back(Point_2(point.x, point.y));
    }
    return polygon;
}

std::vector<Point> RayCasting::convertToVectorPoints(const Polygon_2& polygonCGAL) const {
    std::vector<Point> polygon;
    for (const auto& vertex : polygonCGAL.vertices()) {
        polygon.push_back(Point(CGAL::to_double(vertex.x()), CGAL::to_double(vertex.y())));
    }
    return polygon;
}

Point_2 RayCasting::convertToCGALPoint(const Point& point) const {
    return Point_2(point.x, point.y);
}

Point RayCasting::convertToPoint(const Point_2& point) const {
    return Point(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
}

std::vector<Point> RayCasting::scalePolygon(const std::vector<Point>& input, double offsetDistance) {
    int n = input.size();
    if (n < 3) return input;

    std::vector<Point> scaled;

    for (int i = 0; i < n; ++i) {
        const Point& prev = input[(i - 1 + n) % n];
        const Point& curr = input[i];
        const Point& next = input[(i + 1) % n];

        double dx1 = curr.x - prev.x;
        double dy1 = curr.y - prev.y;
        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        dx1 /= len1;
        dy1 /= len1;

        double dx2 = next.x - curr.x;
        double dy2 = next.y - curr.y;
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        dx2 /= len2;
        dy2 /= len2;

        double nx1 = -dy1, ny1 = dx1;
        double nx2 = -dy2, ny2 = dx2;

        double nx = nx1 + nx2;
        double ny = ny1 + ny2;
        double norm = std::sqrt(nx * nx + ny * ny);
        if (norm < 1e-6) norm = 1.0; 

        nx /= norm;
        ny /= norm;

        double dot = dx1 * dx2 + dy1 * dy2;
        double angleFactor = 1.0 / std::max(0.1, std::sqrt((1 - dot) / 2)); 

        scaled.push_back(Point(
            curr.x + nx * offsetDistance * angleFactor,
            curr.y + ny * offsetDistance * angleFactor
        ));
    }

    return scaled;
}


std::pair<int, Polygon_2> RayCasting::computeVisibilityPolygon2(const int& index, const Point& source) const {
    Point_2 cgal_source(source.x, source.y);
    
    Polygon_2 environmentPolygon;
    for (const auto& p : environment.getPoints()) {
        environmentPolygon.push_back(Point_2(p.x, p.y));
    }

    std::vector<Polygon_2> obstacle_polygons;
    
    for (const auto& obstacle : obstacles.getObstacles()) {
        Polygon_2 obstacle_poly;
        //for (const auto& p : obstacle.getCorners()) {
        for (const auto& p : obstacle.getObstacleCorners()) {
            obstacle_poly.push_back(Point_2(p.x, p.y));
        }
        obstacle_polygons.push_back(obstacle_poly);
    }

    Polygon_with_holes envWithHoles(environmentPolygon, obstacle_polygons.begin(), obstacle_polygons.end());

    Arrangement_2 env;
    
    CGAL::insert_non_intersecting_curves(env, envWithHoles.outer_boundary().edges_begin(), envWithHoles.outer_boundary().edges_end());
    for (const auto& hole : envWithHoles.holes()) {
        CGAL::insert_non_intersecting_curves(env, hole.edges_begin(), hole.edges_end());
    }

    Arrangement_2::Face_const_handle * face;
    CGAL::Arr_naive_point_location<Arrangement_2> pointLocation(env);
    CGAL::Arr_point_location_result<Arrangement_2>::Type result = pointLocation.locate(cgal_source);
    face = std::get_if<Arrangement_2::Face_const_handle> (&result);
    if (face == nullptr) {
        throw std::runtime_error("Source point is outside environment polygon");
    }


    TEV rVisibility(env);
    rVisibility.compute_visibility(cgal_source, *face, env);

    std::pair<int, Polygon_2> visibilityPolygon;
    for (auto fit = env.faces_begin(); fit != env.faces_end(); ++fit) {
        if (!fit->is_unbounded()) {
            Polygon_2 visible_polygon;
            auto circ = fit->outer_ccb();
            auto curr = circ;
            do {
                visible_polygon.push_back(curr->source()->point());
                ++curr;
            } while (curr != circ);

            if (visible_polygon.is_simple() && visible_polygon.is_counterclockwise_oriented()) {
                visibilityPolygon = {index, visible_polygon};
            }
        }
    }
    if (visibilityPolygon.second.is_simple() && visibilityPolygon.second.is_counterclockwise_oriented()) {
        return visibilityPolygon;
    } else {
        throw std::runtime_error("Invalid visibility polygon");
    }
}

bool RayCasting::isPointInsidePolygon(const Point& point, const Polygon_2& polygon) const {
    Point_2 cgal_point(point.x, point.y);
    return polygon.has_on_bounded_side(cgal_point);
}


bool RayCasting::doPolygonOverlayRegion(const Polygon_2& visPolygon, const Polygon_2& region) const {
    Arrangement_2 visPolygon_arr;
    CGAL::insert_non_intersecting_curves(visPolygon_arr, visPolygon.edges_begin(), visPolygon.edges_end());

    Arrangement_2 region_arr;    
    CGAL::insert_non_intersecting_curves(region_arr, region.edges_begin(), region.edges_end());

    Arrangement_2 overlay_arr;
    OverlayTraits overlay_traits;
    
    CGAL::overlay(visPolygon_arr, region_arr, overlay_arr, overlay_traits);
   
    if (overlay_arr.number_of_faces() > 3) {
        return true;
    }
    
    return false;
}

std::map<const Polygon_2*, std::vector<std::pair<int, Polygon_2>>> RayCasting::computeSubregionsSearch(
    const std::vector<Polygon_2>& subregions, 
    const std::vector<std::pair<int, Polygon_2>>& visPolygons
) {
    std::map<const Polygon_2*, std::vector<std::pair<int, Polygon_2>>> subregionMap;
    subregionMap.clear();

    for(const auto& subregion : subregions) {
        std::vector<std::pair<int, Polygon_2>> overlayingPolygons;
        
        for (const auto& visPolygon : visPolygons) {
            if (doPolygonOverlayRegion(visPolygon.second, subregion)) {
                overlayingPolygons.push_back(visPolygon);
            }
        }

        subregionMap.insert({&subregion, overlayingPolygons});
        overlayingPolygons.clear();
    }

    return subregionMap;
}



std::vector<Polygon_2> RayCasting::windowSubdivision(int width, int height, int div){
    std::vector<Polygon_2> subregions;
    float stepX = static_cast<float>(width) / div;
    float stepY = static_cast<float>(height) /div;
    for(int i = 0; i < div; ++i){
        for(int j = 0; j < div; ++j){
            Polygon_2 subregion;
            subregion.push_back(Point_2(i * stepX, j * stepY));
            subregion.push_back(Point_2(i * stepX, (j + 1) * stepY));
            subregion.push_back(Point_2((i + 1) * stepX, (j + 1) * stepY));
            subregion.push_back(Point_2((i + 1) * stepX, j * stepY));
            subregions.push_back(subregion);
        }
    }
    return subregions;
} 

std::vector<std::vector<float>> RayCasting::extendingAdjacencyMatrix(
    const Point_2& start,
    const Point_2& end, 
    const std::vector<std::vector<float>>& adjacencyMatrix,
    const std::vector<Polygon_2>& subregions,
    const std::vector<Point>& searchPoints,
    const std::vector<std::pair<int, Polygon_2>>& visiPolygons
) {
    std::map<const Polygon_2*, std::vector<std::pair<int, Polygon_2>>> subregionMap 
            = computeSubregionsSearch(subregions, visiPolygons);
    
    std::vector<std::vector<float>> extendedMatrix = adjacencyMatrix;
    size_t newSize = adjacencyMatrix.size() + 2;

    Point startF = convertToPoint(start);
    Point endF = convertToPoint(end);
    
    extendedMatrix.resize(newSize);
    for (auto& row : extendedMatrix) {
        row.resize(newSize, 0);
    }
    
    for (const auto& subregion : subregions) {
        if (subregion.has_on_bounded_side(start)) {
            const std::vector<std::pair<int, Polygon_2>>& overlayingPolygons = subregionMap[&subregion];
            for (const auto& polygon : overlayingPolygons) {
                if (polygon.second.has_on_bounded_side(start)) {
                    int polygonIndex = polygon.first;
                    float distance = std::sqrt(
                        std::pow(startF.x - searchPoints[polygonIndex].x, 2) +
                        std::pow(startF.y - searchPoints[polygonIndex].y, 2)
                    );
                    extendedMatrix[polygonIndex][newSize - 2] = distance;
                    extendedMatrix[newSize - 2][polygonIndex] = distance;
                }
            }
        }
        
        if (subregion.has_on_bounded_side(end)) {
            const std::vector<std::pair<int, Polygon_2>>& overlayingPolygons = subregionMap[&subregion];
            for (const auto& polygon : overlayingPolygons) {
                if (polygon.second.has_on_bounded_side(end)) {
                    int polygonIndex = polygon.first;
                    float distance = std::sqrt(
                        std::pow(endF.x - searchPoints[polygonIndex].x, 2) +
                        std::pow(endF.y - searchPoints[polygonIndex].y, 2)
                    );
                    extendedMatrix[polygonIndex][newSize - 1] = distance;  
                    extendedMatrix[newSize - 1][polygonIndex] = distance;
                }
            }
        }
        
        if (subregion.has_on_bounded_side(start) && subregion.has_on_bounded_side(end)) {
            const std::vector<std::pair<int, Polygon_2>>& overlayingPolygons = subregionMap[&subregion];
            for (const auto& polygon : overlayingPolygons) {
                if (polygon.second.has_on_bounded_side(start) && polygon.second.has_on_bounded_side(end)) {
                    float distance = std::sqrt(
                        std::pow(startF.x - endF.x, 2) +
                        std::pow(startF.y - endF.y, 2)
                    );
                    extendedMatrix[newSize - 2][newSize - 1] = distance;
                    extendedMatrix[newSize - 1][newSize - 2] = distance;
                }
            }
        }
    }
    
    return extendedMatrix;
}
