#include "PathFinder.h"
#include <stdexcept>
//#include <iostream>
#include <cmath>
#include <unordered_map>
#include <utility>  // for std::pair

Graph PathFinder::convertMatrixToBoostGraph(
    const std::vector<std::vector<float>>& matrix,
    const std::vector<Point>& searchPoints)
{
    if (matrix.empty() || matrix.size() != searchPoints.size()) {
        throw std::invalid_argument("Matrix size must match searchPoints size");
    }

    // Create graph with vertices for all search points
    Graph g(searchPoints.size());
    
    // Set vertex properties (points)
    for (size_t i = 0; i < searchPoints.size(); ++i) {
        g[i].point = searchPoints[i];
    }

    // Add edges from the matrix (only upper triangle to avoid duplicates)
    for (size_t i = 0; i < matrix.size(); ++i) {
        for (size_t j = i + 1; j < matrix[i].size(); ++j) {
            float weight = matrix[i][j];
            if (weight > 0) {  // Only add edges with positive weights
                auto [edge, added] = boost::add_edge(i, j, g);
                boost::put(boost::edge_weight, g, edge, weight);
            }
        }
    }

    return g;
}

int PathFinder::findPointIndex(const std::vector<Point>& points, const Point& target) {
    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i] == target) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::vector<Point> PathFinder::boostShortestPath(
    const std::vector<std::vector<float>>& matrix,
    const std::vector<Point>& searchPoints,
    const Point& start,
    const Point& end)
{
    // Convert matrix to Boost graph
    Graph g = convertMatrixToBoostGraph(matrix, searchPoints);

    // Find start and end vertices
    int startIdx = findPointIndex(searchPoints, start);
    int endIdx = findPointIndex(searchPoints, end);
    
    if (startIdx == -1 || endIdx == -1) {
        //std::cout << "Start or end point not found in search points.\n";
        return {};  // Either start or end not found in searchPoints
    }

    Vertex startVertex = boost::vertex(startIdx, g);
    Vertex endVertex = boost::vertex(endIdx, g);

    // Prepare for Dijkstra's algorithm
    std::vector<Vertex> predecessors(boost::num_vertices(g));
    std::vector<double> distances(boost::num_vertices(g));

    boost::dijkstra_shortest_paths(
        g, startVertex,
        boost::predecessor_map(&predecessors[0])
         .distance_map(&distances[0])
    );

    // Reconstruct path
    std::vector<Point> path;
    for (Vertex v = endVertex; v != startVertex; v = predecessors[v]) {
        if (predecessors[v] == v) {  // No path exists
            return {};
        }
        path.push_back(g[v].point);
    }
    path.push_back(g[startVertex].point);
    std::reverse(path.begin(), path.end());

    return path;
}







/*Graph convertMatrixToBoostGraph(const std::vector<std::vector<float>>& adjacencyMatrix) {
    // Create graph with the same number of vertices as the matrix size
    Graph g(adjacencyMatrix.size());
    
    // Add edges from the adjacency matrix
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = i; j < adjacencyMatrix[i].size(); ++j) {  // Start from i to avoid duplicates
            float weight = adjacencyMatrix[i][j];
            if (weight > 0 && i != j) {  // Skip diagonal (self-edges) and zero weights
                auto edge = boost::add_edge(i, j, g).first;
                boost::put(boost::edge_weight, g, edge, weight);
            }
        }
    }
    
    // Set vertex indices (important for algorithms)
    auto index_map = boost::get(boost::vertex_index, g);
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        boost::put(index_map, i, i);
    }
    
    return g;
}*/

/*Graph PathFinder::convertToBoostGraph(const VisibilityGraph& visGraph) {
    std::vector<std::pair<size_t, size_t>> edges;  // Store vertex indices
    std::vector<double> weights;
    std::unordered_map<Point, size_t> pointToIndex;
    std::vector<Point> vertices;

    // First pass: collect all unique points and assign indices
    for (const auto& [point, neighbors] : visGraph.adjacency_list) {
        if (pointToIndex.find(point) == pointToIndex.end()) {
            pointToIndex[point] = vertices.size();
            vertices.push_back(point);
        }
        for (const auto& [neighbor, weight] : neighbors) {
            if (pointToIndex.find(neighbor) == pointToIndex.end()) {
                pointToIndex[neighbor] = vertices.size();
                vertices.push_back(neighbor);
            }
        }
    }

    // Second pass: create edges with indices
    for (const auto& [point, neighbors] : visGraph.adjacency_list) {
        for (const auto& [neighbor, weight] : neighbors) {
            // Avoid duplicate edges (since graph is undirected)
            if (pointToIndex[point] < pointToIndex[neighbor]) {
                edges.emplace_back(pointToIndex[point], pointToIndex[neighbor]);
                weights.push_back(weight);
            }
        }
    }

    // Build Boost graph with vertex properties
    Graph g(vertices.size());
    
    // Set vertex properties (Point data)
    for (size_t i = 0; i < vertices.size(); ++i) {
        g[i].point = vertices[i];
    }

    // Add edges with weights
    for (size_t i = 0; i < edges.size(); ++i) {
        auto [u, v] = edges[i];
        auto [edge, added] = boost::add_edge(u, v, g);
        boost::put(boost::edge_weight, g, edge, weights[i]);
    }

    return g;
}

std::vector<Point> PathFinder::boostShortestPath(
    const VisibilityGraph& visGraph,
    const Point& start,
    const Point& end
) {
    Graph g = convertToBoostGraph(visGraph);
    Vertex startVertex = boost::graph_traits<Graph>::null_vertex();
    Vertex endVertex = boost::graph_traits<Graph>::null_vertex();

    // Find start and end vertices
    boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        if (g[*vi].point == start) startVertex = *vi;
        if (g[*vi].point == end) endVertex = *vi;
    }

    if (startVertex == boost::graph_traits<Graph>::null_vertex() ||
        endVertex == boost::graph_traits<Graph>::null_vertex()) {
        return {};  // No path exists
    }

    // Prepare for Dijkstra's algorithm
    std::vector<Vertex> predecessors(boost::num_vertices(g));
    std::vector<double> distances(boost::num_vertices(g));

    // Run Dijkstra's
    boost::dijkstra_shortest_paths(
        g, startVertex,
        boost::predecessor_map(&predecessors[0])
         .distance_map(&distances[0])
    );

    // Reconstruct path
    std::vector<Point> path;
    for (Vertex v = endVertex; v != startVertex; v = predecessors[v]) {
        path.push_back(g[v].point);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}*/
// Boost Graph Library types
/*typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, float>> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef std::pair<int, int> EdgePair;

float PathFinder::computeDistance(const Point& p1, const Point& p2) const {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

std::vector<Point> PathFinder::findShortestPath(
    const std::vector<Point>& points,
    const std::vector<std::vector<int>>& visibilityMatrix,
    const Point& start,
    const Point& end
) const {
    // Map points to indices
    std::unordered_map<Point, size_t> pointToIndex;
    for (size_t i = 0; i < points.size(); ++i) {
        pointToIndex[points[i]] = i;
    }

    // Check if start and end points are in the graph
    if (pointToIndex.find(start) == pointToIndex.end() || pointToIndex.find(end) == pointToIndex.end()) {
        return {}; // No path exists
    }

    size_t startIndex = pointToIndex[start];
    size_t endIndex = pointToIndex[end];

    // Create a Boost graph
    Graph g;

    // Add edges to the graph based on visibility
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            if (visibilityMatrix[i][j] == 1) { // If points i and j are visible
                float weight = computeDistance(points[i], points[j]); // Euclidean distance
                boost::add_edge(i, j, weight, g);
            }
        }
    }

    // Run Dijkstra's algorithm
    std::vector<Vertex> predecessors(boost::num_vertices(g));
    std::vector<float> distances(boost::num_vertices(g));
    boost::dijkstra_shortest_paths(
        g, startIndex,
        boost::predecessor_map(&predecessors[0]).distance_map(&distances[0])
    );

    // Reconstruct the path
    std::vector<Point> path;
    for (Vertex at = endIndex; at != startIndex; at = predecessors[at]) {
        if (predecessors[at] == at) {
            return {}; // No path exists
        }
        path.push_back(points[at]);
    }
    path.push_back(points[startIndex]);
    std::reverse(path.begin(), path.end());

    return path;
}*/


/*
I will like you to convert this adjacency matrix of the form std::vector<std::vector<float>> containing distance of related points in to the boost graph I gave you:

Extended Matrix: 
Visibility Adjacency Matrix:
    P0 P1 P2 P3 P4 P5 P6 P7 P8 P9 P10 P11 P12 P13 P14 P15 P16 P17 P18 P19 P20 P21 P22 P23 P24 P25 P26 P27 P28 P29 P30 P31 P32 P33 P34 P35 P36 P37 P38 P39 P40 P41 
P0  0  31  0  31  0  0  0  580.524  199.381  0  221.285  195.376  0  0  0  0  317.504  0  330.024  310.573  0  0  536.474  0  242.022  0  238.708  224.935  310.941  0  320.93  293.915  392.444  0  403.926  381.549  500.9  0  503.723  481.44  0  0 
P1  31  0  31  0  0  0  0  0  169.259  0  190.397  164.523  467.291  0  0  0  288.907  0  300.515  281.272  519.722  0  515.763  499.588  225.615  0  219.063  207.179  286.35  0  293.915  267.765  365.368  0  375.657  353.639  478.846  0  479.929  458.451  0  134.471 
P2  0  31  0  31  0  0  606.192  0  179.723  0  195.68  170.609  0  0  0  0  302.781  0  0  293.356  544.892  0  539.66  524.223  253.251  0  244.748  234.172  306.947  0  310.941  286.35  381.948  0  389.859  368.69  501.586  0  500.829  480.286  0  129.778 
P3  31  0  31  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  160.763 
P4  0  0  0  0  0  31  0  31  483.512  0  0  0  0  0  0  0  391.69  390.27  410.652  0  0  0  0  0  364.38  353.539  372.291  0  334.943  328.901  359.582  0  0  0  0  0  0  0  0  0  28.86  568.893 
P5  0  0  0  0  31  0  31  0  0  0  0  0  0  0  0  0  0  0  0  0  0  90.1641  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  165.335  0  0  35.9402  0 
P6  0  0  606.192  0  0  31  0  31  464.253  0  0  0  0  0  0  0  0  0  0  0  63.062  59.1977  0  0  357.118  344.137  361.552  0  314.514  0  0  0  0  0  0  0  0  134.942  0  0  0  550.087 
P7  580.524  0  0  0  31  0  31  0  453.887  0  0  0  0  0  0  0  360.843  359.302  379.681  0  59.7814  68.3825  0  84.9779  338.504  326.806  345.097  0  304.936  298.286  328.901  0  291.779  0  0  316.253  0  0  0  0  0  539.423 
P8  199.381  169.259  179.723  0  483.512  0  464.253  453.887  0  26  0  26  0  0  0  0  124.59  0  132.139  113.754  401.201  0  0  377.682  0  158.256  137.532  144.289  149.8  0  139.202  121.789  207.184  0  0  191.361  0  0  0  0  512.371  85.8745 
P9  0  0  0  0  0  0  0  0  26  0  26  0  0  0  0  0  104.725  0  108.833  91.5684  0  0  0  0  0  165.806  146.156  156.12  141.811  0  124.24  111.817  188.958  0  0  0  0  0  321.78  0  0  0 
P10  221.285  190.397  195.68  0  0  0  0  0  0  26  0  26  0  0  307.275  0  124.36  0  123.62  108.729  0  0  0  0  0  0  0  0  167.251  0  147.262  136.943  209.071  0  0  0  0  0  344.016  330.437  0  79.9151 
P11  195.376  164.523  170.609  0  0  0  0  0  26  0  26  0  0  0  0  0  0  0  0  0  0  0  0  0  0  183.733  162.849  168.595  0  0  0  0  0  0  0  0  0  0  0  0  0  62.8301 
P12  0  467.291  0  0  0  0  0  0  0  0  0  0  0  24.2  0  24.2  0  0  0  0  0  192.603  0  0  297.42  276.31  280.055  0  189.822  160.38  173.401  0  101.924  81.7862  99.1458  0  0  115.498  89.0093  105.609  0  0 
P13  0  0  0  0  0  0  0  0  0  0  0  0  24.2  0  24.2  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  126.173  102.481  123.031  0  0 
P14  0  0  0  0  0  0  0  0  0  0  307.275  0  0  24.2  0  24.2  0  171.569  183.663  0  0  0  0  0  319.459  298.181  299.94  0  208.231  177.621  0  0  115.314  91.717  101.819  0  0  0  0  0  0  387.145 
P15  0  0  0  0  0  0  0  0  0  0  0  0  24.2  0  24.2  0  0  0  165.548  0  0  0  0  0  295.284  274.01  275.924  0  184.324  153.834  0  0  92.0032  69.0296  81.9767  0  0  138.376  111.027  124.73  0  0 
P16  317.504  288.907  302.781  0  391.69  0  0  360.843  124.59  104.725  124.36  0  0  0  0  0  0  20.4  0  20.4  303.369  0  283.638  278.295  0  157.161  147.312  167.004  0  65.7672  35.7061  54.475  84.7116  0  87.0791  66.8675  237.388  0  221.016  209.801  0  0 
P17  0  0  0  0  390.27  0  0  359.302  0  0  0  0  0  0  171.569  0  20.4  0  20.4  0  300.925  0  279.173  275.629  0  174.956  166.164  186.197  0  72.2698  46.6104  71.6235  74.1702  0  70.0594  52.8812  232.298  0  212.777  204.024  0  0 
P18  330.024  300.515  0  0  410.652  0  0  379.681  132.139  108.833  123.62  0  0  0  183.663  165.548  0  20.4  0  20.4  321.254  0  299.247  295.944  0  0  0  0  0  0  0  0  93.1093  0  84.124  70.4624  252.286  0  231.898  223.894  0  203.514 
P19  310.573  281.272  293.356  0  0  0  0  0  113.754  91.5684  108.729  0  0  0  0  0  20.4  0  20.4  0  0  0  0  0  0  168.59  156.699  175.341  0  85.9142  55.3719  68.983  0  0  0  0  0  0  0  0  0  0 
P20  0  519.722  544.892  0  0  0  63.062  59.7814  401.201  0  0  0  0  0  0  0  303.369  300.925  321.254  0  0  25.4  0  25.4  297.988  283.955  300.671  0  251.453  242.197  272.39  0  0  0  0  256.896  0  0  0  0  0  487.042 
P21  0  0  0  0  0  90.1641  59.1977  68.3825  0  0  0  0  192.603  0  0  0  0  0  0  0  25.4  0  25.4  0  0  0  0  0  0  0  0  0  0  0  0  0  0  77.1152  0  0  0  0 
P22  536.474  515.763  539.66  0  0  0  0  0  0  0  0  0  0  0  0  0  283.638  279.173  299.247  0  0  25.4  0  25.4  298.755  282.853  297.819  0  238.971  225.75  254.805  0  207.968  0  0  0  47.2952  53.8873  0  0  0  0 
P23  0  499.588  524.223  0  0  0  0  84.9779  377.682  0  0  0  0  0  0  0  278.295  275.629  295.944  0  25.4  0  25.4  0  279.875  264.884  280.809  0  227.883  217.627  247.636  0  207.007  209.021  0  231.501  52.2456  69.7839  0  79.4242  0  463.555 
P24  242.022  225.615  253.251  0  364.38  0  357.118  338.504  0  0  0  0  297.42  0  319.459  295.284  0  0  0  0  297.988  0  298.755  279.875  0  21.3  0  21.3  0  0  0  0  0  0  0  0  268.09  0  279.487  253.45  392.235  0 
P25  0  0  0  0  353.539  0  344.137  326.806  158.256  165.806  0  183.733  276.31  0  298.181  274.01  157.161  174.956  0  168.59  283.955  0  282.853  264.884  21.3  0  21.3  0  93.2939  123.933  0  103.337  186.397  210.839  0  0  250.503  0  260.302  234.768  381.733  233.45 
P26  238.708  219.063  244.748  0  372.291  0  361.552  345.097  137.532  146.156  0  162.849  280.055  0  299.94  275.924  147.312  166.164  0  156.699  300.671  0  297.819  280.809  0  21.3  0  21.3  91.9833  0  0  95.4727  0  0  0  0  263.498  0  270.558  246.09  400.638  212.15 
P27  224.935  207.179  234.172  0  0  0  0  0  144.289  156.12  0  168.595  0  0  0  0  167.004  186.197  0  175.341  0  0  0  0  21.3  0  21.3  0  0  0  0  116.11  0  0  0  0  0  0  0  0  0  213.232 
P28  310.941  286.35  306.947  0  334.943  0  314.514  304.936  149.8  141.811  167.251  0  189.822  0  208.231  184.324  0  0  0  0  251.453  0  238.971  227.883  0  93.2939  91.9833  0  0  31  0  31  94.0902  0  0  0  197.131  0  193.883  174.002  0  235.672 
P29  0  0  0  0  328.901  0  0  298.286  0  0  0  0  160.38  0  177.621  153.834  65.7672  72.2698  0  85.9142  242.197  0  225.75  217.627  0  123.933  0  0  31  0  31  0  63.1099  0  0  66.6961  181.391  0  172.726  155.944  0  0 
P30  320.93  293.915  310.941  0  359.582  0  0  328.901  139.202  124.24  147.262  0  173.401  0  0  0  35.7061  46.6104  0  55.3719  272.39  0  254.805  247.636  0  0  0  0  0  31  0  31  71.5148  0  88.0373  63.7249  209.615  0  197.667  183.185  0  0 
P31  293.915  267.765  286.35  0  0  0  0  0  121.789  111.817  136.943  0  0  0  0  0  54.475  71.6235  0  68.983  0  0  0  0  0  103.337  95.4727  116.11  31  0  31  0  0  0  0  0  0  0  0  0  0  0 
P32  392.444  365.368  381.948  0  0  0  0  291.779  207.184  188.958  209.071  0  101.924  0  115.314  92.0032  84.7116  74.1702  93.1093  0  0  0  207.968  207.007  0  186.397  0  0  94.0902  63.1099  71.5148  0  0  24.5  0  24.5  160.711  0  138.789  131.923  0  0 
P33  0  0  0  0  0  0  0  0  0  0  0  0  81.7862  0  91.717  69.0296  0  0  0  0  0  0  0  209.021  0  210.839  0  0  0  0  0  0  24.5  0  24.5  0  0  0  132.596  130.935  0  0 
P34  403.926  375.657  389.859  0  0  0  0  0  0  0  0  0  99.1458  0  101.819  81.9767  87.0791  70.0594  84.124  0  0  0  0  0  0  0  0  0  0  0  88.0373  0  0  24.5  0  24.5  0  0  156.808  0  0  0 
P35  381.549  353.639  368.69  0  0  0  0  316.253  191.361  0  0  0  0  0  0  0  66.8675  52.8812  70.4624  0  256.896  0  0  231.501  0  0  0  0  0  66.6961  63.7249  0  24.5  0  24.5  0  0  0  0  0  0  0 
P36  500.9  478.846  501.586  0  0  0  0  0  0  0  0  0  0  0  0  0  237.388  232.298  252.286  0  0  0  47.2952  52.2456  268.09  250.503  263.498  0  197.131  181.391  209.615  0  160.711  0  0  0  0  29  0  29  0  0 
P37  0  0  0  0  0  165.335  134.942  0  0  0  0  0  115.498  126.173  0  138.376  0  0  0  0  0  77.1152  53.8873  69.7839  0  0  0  0  0  0  0  0  0  0  0  0  29  0  29  0  0  0 
P38  503.723  479.929  500.829  0  0  0  0  0  0  321.78  344.016  0  89.0093  102.481  0  111.027  221.016  212.777  231.898  0  0  0  0  0  279.487  260.302  270.558  0  193.883  172.726  197.667  0  138.789  132.596  156.808  0  0  29  0  29  0  0 
P39  481.44  458.451  480.286  0  0  0  0  0  0  0  330.437  0  105.609  123.031  0  124.73  209.801  204.024  223.894  0  0  0  0  79.4242  253.45  234.768  246.09  0  174.002  155.944  183.185  0  131.923  130.935  0  0  29  0  29  0  0  0 
P40  0  0  0  0  28.86  35.9402  0  0  512.371  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  392.235  381.733  400.638  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0 
P41  0  134.471  129.778  160.763  568.893  0  550.087  539.423  85.8745  0  79.9151  62.8301  0  0  387.145  0  0  0  203.514  0  487.042  0  0  463.555  0  233.45  212.15  213.232  235.672  0  0  0  0  0  0  0  0  0  0  0  0  0
*/