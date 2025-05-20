#ifndef RAYCASTING_H
#define RAYCASTING_H

#include <vector>
#include <map>
#include <utility>
#include <stdexcept>
#include <iostream>
#include "utils/Point.h"
#include "environment/Environment.h"
#include "obstacles/Obstacles.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Simple_polygon_visibility_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Gps_traits_2.h>

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_naive_point_location.h>

#include <CGAL/Arr_overlay_2.h>
#include <CGAL/basic.h>
#include <CGAL/Arr_default_overlay_traits.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;

typedef CGAL::Point_2<Kernel>                             Point_2;
typedef CGAL::Segment_2<Kernel>                           Segment_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;

typedef CGAL::Polygon_with_holes_2<Kernel>               Polygon_with_holes;

typedef CGAL::Arr_segment_traits_2<Kernel>               Traits;
typedef CGAL::Arrangement_2<Traits> Arrangement_2;
typedef Arrangement_2::Face_handle     Face_handle;
typedef Arrangement_2::Edge_const_iterator    Edge_const_iterator;
typedef Arrangement_2::Ccb_halfedge_circulator   Ccb_halfedge_circulator;

typedef CGAL::Simple_polygon_visibility_2<Arrangement_2, CGAL::Tag_true>        Regularized_visibility;
typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>  TEV;
typedef boost::shared_ptr<Polygon_with_holes>           PolygonPtr;

typedef CGAL::Arr_default_overlay_traits<Arrangement_2> OverlayTraits;

class RayCasting {
public:
    RayCasting(const Environment& environment, const Obstacles& obstacles);

    bool isPointInsideObstacle(const Point& point) const;

    bool castRay(const Point& source, const Point& target, const Obstacle* excludeObstacle = nullptr) const;

    std::vector<Point> computeVisibility(const Point& source, const std::vector<Point>& targetPoints, const Obstacle* excludeObstacle = nullptr);

    std::vector<std::vector<int>> getVisibilityMatrix(const std::vector<Point>& points) const;

    std::vector<int> getAdjacencyColumn(const Point& source, const std::vector<Point>& targetPoints, const Obstacle* excludeObstacle= nullptr) const;

    std::vector<std::vector<Point*>> getVisiVectors(const Point& source, const std::vector<Point>& targetPoints, const Obstacle* excludeObstacle = nullptr) const;

    void printVisibilityMatrix(const std::vector<Point>& points) const;
    
    void printAdjacencyColumn(const Point& origin, const std::vector<int>& column);

    std::vector<Point> getVisiblePoints(const Point& source) const;

    std::vector<Point> getVisiblePointsByType(const Point& source, const std::string& type) const;

    Polygon_2 convertToCGALPolygon(const std::vector<Point>& points) const;

    std::vector<Point> convertToVectorPoints(const Polygon_2& polygon) const;

    Point_2 convertToCGALPoint(const Point& point) const;

    Point convertToPoint(const Point_2& point) const;

    std::pair<int, Polygon_2> computeVisibilityPolygon2(const int& index, const Point& source) const;

    std::vector<Point> scalePolygon(const std::vector<Point>& input, double offsetDistance);

    bool isPointInsidePolygon(const Point& point, const Polygon_2& polygon) const;

    bool doPolygonOverlayRegion(const Polygon_2& polygon, const Polygon_2& region) const;
    
    std::map<const Polygon_2*, std::vector<std::pair<int, Polygon_2>>> computeSubregionsSearch(const std::vector<Polygon_2>& subregions, const std::vector<std::pair<int, Polygon_2>>& visPolygons);

    std::vector<Polygon_2> windowSubdivision(int width, int height, int div);

    std::vector<std::vector<float>> extendingAdjacencyMatrix(
        const Point_2& start,
        const Point_2& end, 
        const std::vector<std::vector<float>>& adjacencyMatrix,
        const std::vector<Polygon_2>& subregions,
        const std::vector<Point>& searchPoints,
        const std::vector<std::pair<int, Polygon_2>>& visiPolygons
    );
    std::vector<std::vector<float>> computeAdjacencyMatrix(
        const std::vector<Point>& points, 
        const Obstacle* excludeObstacle
    ) const;

    bool isPointInsideCGALObstacle(const Point_2& point) const;

    void printFloatMatrix(const std::vector<std::vector<float>>& visibilityMatrix) const;

private:
    const Environment& environment; 
    const Obstacles& obstacles; 

    std::map<Point, std::vector<Point>> visibilityPoints;
};

#endif 