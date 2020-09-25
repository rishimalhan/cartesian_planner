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

#ifdef PCT_PLANNER
#include <pct/sample_nodes.hpp>
#endif

bool isEdge(const std::vector<node*>& node_map, const int parent, const int child){

    if (node_map[parent]->family_id != node_map[child]->family_id)
        return false;
    return true;
}


double computeGCost( const std::vector<node*>& node_map, const int parent, const int child ){
    return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
    // return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
    // return (node_map[child]->jt_config - node_map[parent]->jt_config).norm();
}




#ifdef BASELINE
// Graph is boost library compatible
// Graphs equal to the number of root nodes are generated
// CAUTION: root_id is the node id which is used to access node_map.
// This code assumes that all nodes are valid within joint limits of robot
bool build_graph(ikHandler* ik_handler, std::vector<Eigen::MatrixXd>& ff_frames,
                    WM::WM* wm,
                    const std::vector<node*>& node_map, const std::vector<Eigen::VectorXi>& node_list,
                    boost_graph* boost_graph, std::vector<bool>& root_connectivity){
    boost_graph->no_nodes = node_map.size();
    std::cout<< "\n##############################################################\n";
    std::cout<< "Generating Graph\n";

    int no_connections_eval = 0;
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
                no_connections_eval++;
                if ( isEdge(node_map, curr_level(j), next_level(k)) ){
                    if (i==0) // Mark this root to be connected to graph
                        root_connectivity[j] = true;
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
#endif // End baseline functions








#ifdef PCT_PLANNER
bool InsertNode( Eigen::VectorXi parents, Eigen::VectorXi children, 
                    std::vector<Edge>& edges, std::vector<double>& weights,
                    ikHandler* ik_handler, const std::vector<node*>& node_map,
                    int& no_connections_eval ){
    bool atleast_one_edge;
    for (int i=0; i<parents.size(); ++i){
        for (int j=0; j<children.size(); ++j){
            if ( isEdge(node_map, parents(i), children(j)) ){
                edges.push_back( Edge(parents(i),children(j)) );
                weights.push_back( computeGCost(node_map, parents(i), children(j)) );
                atleast_one_edge = true;
            }
            no_connections_eval++;
        }
    }
    atleast_one_edge = false;
    return atleast_one_edge;
}

// Graph is boost library compatible
// Graphs equal to the number of root nodes are generated
// CAUTION: root_id is the node id which is used to access node_map.
// This code assumes that all nodes are valid within joint limits of robot
bool build_graph(ikHandler* ik_handler, std::vector<Eigen::MatrixXd>& ff_frames,
                    WM::WM* wm,
                    std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                    boost_graph* boost_graph, std::vector<bool>& root_connectivity){
    
    std::cout<< "\n##############################################################\n";
    std::cout<< "Generating Graph\n";
    node_map.clear();
    node_list.clear();
    int no_connections_eval = 0;
    std::vector<Edge> edges; edges.clear();
    std::vector<double> weights; weights.clear();
    root_connectivity.resize(ff_frames[0].size());
    for (int i=0; i<ff_frames[0].size(); ++i)
        root_connectivity[i] = false;
    std::cout<< "Number of levels in the graph: " << ff_frames.size() << "\n";
    bool stopping_condition = false;
    std::vector<std::vector<int>> unvisited_ids(ff_frames.size());
    for (int i=0; i<ff_frames.size();++i){
        for (int j=0; j<ff_frames[i].rows();++j)
            unvisited_ids[i].push_back(j);
    }
    int itr = 0;
    int max_itr = 50;
    while (!stopping_condition){
        bool samples_gen = false;
        // Iterate through each level
        for (int depth=0; depth<ff_frames.size(); ++depth){
            // Sample nodes to be added to graph
            std::vector<int> sampled_nodes;
            if( !GenNodeSamples(ff_frames, ik_handler, wm, sampled_nodes, unvisited_ids, 
                             node_map, node_list, depth) )
                continue;
            // Discontinuity checker goes here
            samples_gen = true;
            // Integrate nodes with graph
            for (auto node_id : sampled_nodes){
                Eigen::VectorXi curr_level(1);
                curr_level(0) = node_id;
                // Connect to next level
                if (depth+1<node_list.size()){
                    InsertNode( curr_level,node_list[depth+1],edges, 
                        weights,ik_handler,node_map, no_connections_eval );
                }
                // Connect to previous level
                if (depth-1>=0){
                    InsertNode( node_list[depth-1],curr_level,edges, 
                        weights,ik_handler,node_map, no_connections_eval );    
                }
            }
        }
        itr++;
        if (!samples_gen)
            stopping_condition = true;
    }
    boost_graph->leaf_nodes = node_list[node_list.size()-1];
    boost_graph->no_nodes = node_map.size();
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

    for (int i=0; i<node_list[0].size();++i){
        if (out_degree(node_list[0](i),boost_graph->g)>0)
            root_connectivity[i] = true;
    }

    return true;
};
#endif // End PCT functionality




// End header
#endif