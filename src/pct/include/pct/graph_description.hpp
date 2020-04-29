///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __graph_description_hpp__
#define __graph_description_hpp__

#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include <Eigen/Eigen>

using namespace boost;

typedef adjacency_list < vecS, vecS, directedS,
        no_property, property < edge_weight_t, double > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef std::pair<int, int> Edge;

struct boost_graph{
    graph_t g;
    std::vector<vertex_descriptor> p;
    std::vector<double> d;
    vertex_descriptor s;
    Eigen::VectorXi leaf_nodes;
    int no_levels;
    int no_edges;
    int no_nodes;
    Eigen::MatrixXi paths; // Each column represents a path. Might not be lowest cost
};

#endif