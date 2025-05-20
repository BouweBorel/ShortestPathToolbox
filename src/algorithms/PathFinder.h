#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include "../utils/Point.h"
#include "../utils/VisibilityGraph.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>

struct VertexProperty {
    Point point;
    VertexProperty() : point(0, 0) {}
    VertexProperty(float x, float y) : point(x,y){}
};


// Define Boost graph types
using Graph = boost::adjacency_list<
    boost::vecS,          // Store edges in a vector
    boost::vecS,          // Store vertices in a vector
    boost::undirectedS,   // Undirected graph
    VertexProperty,       // Vertex properties (now stores Point)
    boost::property<boost::edge_weight_t, double>  // Edge weight (distance)
>;

using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

class PathFinder {
public:
    // Convert extended adjacency matrix to Boost graph
    Graph convertMatrixToBoostGraph(
        const std::vector<std::vector<float>>& matrix,
        const std::vector<Point>& searchPoints
    );

    // Find shortest path using extended matrix
    std::vector<Point> boostShortestPath(
        const std::vector<std::vector<float>>& matrix,
        const std::vector<Point>& searchPoints,
        const Point& start,
        const Point& end
    );

    /*template <typename Graph>
    void getBoostGraph(const std::vector<std::vector<float>>& adjacencyMatrix, 
                      const std::vector<Point>& points, 
                      Graph& g) const {
        typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
        
        // Clear the graph
        g.clear();
        
        // Add vertices
        std::vector<Vertex> vertices(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            vertices[i] = boost::add_vertex(g);
            g[vertices[i]].point = points[i];
        }
        
        // Add edges based on adjacency matrix
        for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
            for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
                if (adjacencyMatrix[i][j] > 0) {  // If there's a connection
                    auto edge = boost::add_edge(vertices[i], vertices[j], g).first;
                    g[edge].weight = adjacencyMatrix[i][j];
                }
            }
        }
    }*/

private:
    // Helper to find index of a point in searchPoints
    int findPointIndex(const std::vector<Point>& points, const Point& target);
};

/*using Edge = boost::graph_traits<Graph>::edge_descriptor;

class PathFinder {
public:

    //Converting visibility graph to boost graph format
    Graph convertToBoostGraph(const VisibilityGraph& visGraph);

    // Find the shortest path between two points using Boost's Dijkstra's algorithm
    std::vector<Point> boostShortestPath(
        const VisibilityGraph& visGraph,
        const Point& start,
        const Point& end
    );
     
    /*std::vector<Point> findShortestPath(
        const std::vector<Point>& points,
        const std::vector<std::vector<int>>& visibilityMatrix,
        const Point& start,
        const Point& end
    ) const;* /

private:
    // Helper function to compute Euclidean distance between two points
    //float computeDistance(const Point& p1, const Point& p2) const;
};*/

#endif // PATHFINDER_H