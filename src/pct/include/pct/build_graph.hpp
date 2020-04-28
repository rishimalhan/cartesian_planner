///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __build_graph_hpp__
#define __build_graph_hpp__

#include <robot_utilities/ikHandler.hpp>
#include <pct/node_description.hpp>
#include <pct/graph_description.hpp>
#include <Eigen/Eigen>
#include <pct/path_consistency.hpp>


bool isEdge(ikHandler* ik_handler, const std::vector<node*>& node_map, const int parent, const int child){
    // Path-Consistency Constraint
    // std::vector<Eigen::VectorXd> seg(4); // x1,q1,x2,q2
    // seg[0] = node_map[parent]->wp;
    // seg[1] = node_map[parent]->jt_config;
    // seg[2] = node_map[child]->wp;
    // seg[3] = node_map[child]->jt_config;

    // if (!path_consistency(seg, ik_handler, 0, get_dist(seg, ik_handler))) // Returns true if jt configs are path consistent
    //     return false;
    return true;
}


double computeGCost( const std::vector<node*>& node_map, const int parent, const int child ){
    return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
    // return (node_map[child]->jt_config - node_map[parent]->jt_config).norm();
}


// Graph is boost library compatible
// Graphs equal to the number of root nodes are generated
// CAUTION: root_id is the node id which is used to access node_map.
// This code assumes that all nodes are valid within joint limits of robot
bool build_graph(ikHandler* ik_handler, const std::vector<node*>& node_map, const std::vector<Eigen::VectorXi>& node_list,
                    boost_graph* boost_graph, std::vector<bool>& root_connectivity){
    boost_graph->no_nodes = node_map.size();
    std::cout<< "\n##############################################################\n";
    std::cout<< "Generating Graph\n";

    std::vector<Edge> edges; edges.clear();
    std::vector<double> weights; weights.clear();
    root_connectivity.resize(node_list[0].size());

    for (int i=0; i<node_list[0].size(); ++i)
        root_connectivity[i] = false;
    std::cout<< "Number of levels in the graph: " << node_list.size() << "\n";

    for (int i=0; i<node_list.size()-1; ++i){
        // Nodes at the current level
        Eigen::VectorXi curr_level = node_list[i];
        // Nodes at the next level
        Eigen::VectorXi next_level = node_list[i+1];
        // Store the leaf nodes for generating path
        if (i==node_list.size()-2)
            boost_graph->leaf_nodes = node_list[i+1];

        int prev_edge_size = edges.size(); // This is a checking for discontinuity; 

        // For each node at current level, build edges for each node at next level
        for (int j=0; j<curr_level.size(); ++j){
            for (int k=0; k<next_level.size(); ++k){
                if ( isEdge(ik_handler, node_map, curr_level(j), next_level(k)) ){
                    if (i==0) // Mark this root to be connected to graph
                        root_connectivity[j] = true;
                    edges.push_back( Edge(curr_level(j),next_level(k)) );
                    weights.push_back( computeGCost(node_map, curr_level(j), next_level(k)) );
                }
            }
        }
        if (prev_edge_size==edges.size()){ // No edges have been added
            std::cout<< "Edges could not be created between: " << i << " and "<< i+1 << ". Base index is 0. Terminating\n";
            std::cout<< "##############################################################\n";
            return false;
        }
        if (edges.size() > 35000000){ // Safe limit for not blowing up memory
            std::cout<< "\n!!!CAUTION!!!\n";
            std::cout<< "Number of edges exceed the threshold of 35 million.\n";
            std::cout<< "Memory space more than 5 gb will be required.\n";
            std::cout<< "Next safe limit is 50 million with 8 gb memory space.\n";
            std::cout<< "Terminating Planning\n";
            std::cout<< "##############################################################\n\n";
            return false;
        }
    }   
    std::cout<< "Total Number of Edges: " << edges.size() << "\n";
    boost_graph->no_edges = edges.size();

    const int num_nodes = node_map.size();
    int num_arcs = edges.size();
    graph_t g(edges.begin(), edges.end(), weights.begin(), num_nodes); boost_graph->g = g;
    std::vector<vertex_descriptor> p(num_vertices(g)); boost_graph->p = p;
    std::vector<double> d(num_vertices(g)); boost_graph->d = d;

    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, boost_graph->g);

    std::cout<< "##############################################################\n";

    boost_graph->no_levels = node_list.size();
    return true;
}


#endif