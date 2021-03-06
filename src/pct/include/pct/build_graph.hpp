///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __build_graph_hpp__
#define __build_graph_hpp__

#include <pct/node_description.hpp>
#include <pct/graph_description.hpp>
#include <Eigen/Eigen>
#include <robot_utilities/world_manager.hpp>
#include <pct/geometric_filter.h>

bool isEdge(const std::vector<node*>& node_map, const int parent, const int child){

    if ((node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff() > 0.87)
        return false;
    return true;
}


double computeGCost( const std::vector<node*>& node_map, const int parent, const int child ){
    // return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
    // return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
    return (node_map[child]->jt_config - node_map[parent]->jt_config).norm();
}


// Graph is boost library compatible
// Graphs equal to the number of root nodes are generated
// CAUTION: root_id is the node id which is used to access node_map.
// This code assumes that all nodes are valid within joint limits of robot
bool build_graph(ikHandler* ik_handler, std::vector<Eigen::MatrixXd>& ff_frames,
                    WM::WM* wm, GeometricFilterHarness* geo_filter,
                    const std::vector<node*>& node_map, const std::vector<Eigen::VectorXi>& node_list,
                    boost_graph* boost_graph, std::vector<bool>& root_connectivity){
    boost_graph->no_nodes = node_map.size();
    std::cout<< "\n##############################################################\n";
    std::cout<< "Generating Graph\n";
    boost_graph->root_connected = true;
    boost_graph->leaf_connected = true;
    
    int no_connections_eval = 0;
    std::vector<Edge> edges; edges.clear();
    std::vector<double> weights; weights.clear();
    // root_connectivity.resize(node_list[0].size());
    // for (int i=0; i<node_list[0].size(); ++i)
    //     root_connectivity[i] = false;
    std::cout<< "Number of levels in the graph: " << node_list.size() << "\n";

    for (int i=0; i<node_list.size(); ++i){
        // Connect dummy leaf
        if (i==node_list.size()-1){
            Eigen::VectorXi curr_level = node_list[i];
            for (int j=0; j<curr_level.size(); ++j){
                edges.push_back( Edge(curr_level(j),1) );
                weights.push_back( 0 );
            }
            continue;
        }

        // Nodes at the current level
        Eigen::VectorXi curr_level = node_list[i];
        // Nodes at the next level
        Eigen::VectorXi next_level = node_list[i+1];
        int prev_edge_size = edges.size(); // This is a checking for discontinuity; 

        // For each node at current level, build edges for each node at next level
        for (int j=0; j<curr_level.size(); ++j){
            // Connect dummy root
            if (i==0){
                edges.push_back( Edge(0,curr_level(j)) );
                weights.push_back( 0 );
            }

            for (int k=0; k<next_level.size(); ++k){
                no_connections_eval++;
                if ( isEdge(node_map, curr_level(j), next_level(k)) ){
                    // if (i==0) // Mark this root to be connected to graph
                    //     root_connectivity[j] = true;
                    edges.push_back( Edge(curr_level(j),next_level(k)) );
                    weights.push_back( computeGCost(node_map, curr_level(j), next_level(k)) );
                }
            }
        }
        if (prev_edge_size==edges.size()){ // No edges have been added
            // This is where we split the graph into two
            // Will be a part of our approach
            std::cout<< "Edges could not be created between: " << i << " and "<< i+1 << ". Base index is 0. Terminating\n";
            std::cout<< "##############################################################\n";
            return false;
        }
        if (edges.size() > 50000000){ // Safe limit for not blowing up memory
            std::cout<< "\n!!!CAUTION!!!\n";
            std::cout<< "Number of edges exceed the threshold of 35 million.\n";
            std::cout<< "Memory space more than 5 gb will be required.\n";
            std::cout<< "Next safe limit is 50 million with 8 gb memory space.\n";
            std::cout<< "Terminating Planning\n";
            std::cout<< "##############################################################\n\n";
            return false;
        }
    } 
    
    std::cout<< "Total #Edges: " << no_connections_eval << ". Valid #edges: " << edges.size() << "\n";
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
};
#endif