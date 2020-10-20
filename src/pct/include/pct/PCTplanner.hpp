///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __PCT_PLANNER_HPP
#define __PCT_PLANNER_HPP

#include <pct/node_description.hpp>
#include <pct/graph_description.hpp>
#include <Eigen/Eigen>
#include <robot_utilities/world_manager.hpp>
#include <pct/geometric_filter.h>
#include <pct/ActionModule.hpp>


bool BuildRefineGraph(ikHandler* ik_handler, std::vector<Eigen::MatrixXd>& ff_frames,
                    WM::WM* wm, GeometricFilterHarness* geo_filter,
                    std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                    boost_graph* boost_graph){
    std::cout<< "\n##############################################################\n";
    std::cout<< "Generating Graph\n";
    ik_handler->setTcpFrame(Eigen::MatrixXd::Identity(4,4));
    node_list.resize(ff_frames.size());
    node_map.clear();
    std::vector<Edge> edges; edges.clear();
    std::vector<double> weights; weights.clear();

    std::cout<< "Number of levels in the graph: " << ff_frames.size() << "\n";
    bool stopping_condition = false;
    int itr = 0;
    int max_itr;
    if(!ros::param::get("/max_pct_itr",max_itr)){
        std::cout<< "Unable to Obtain Maximum Iterations\n";
        return 0;
    }
    
    Actions actions(ff_frames);

    graph_t g;
    boost_graph->g = g;

    // Determine Conditions and assign probability to actions
    std::vector<double> prob_range(3);
    prob_range[0] = 1.0;
    prob_range[1] = 1.0;
    prob_range[2] = 1.0;

    // Take action
    while (itr<max_itr){
        // Sample from distribution
        double probability = (double) std::rand() / RAND_MAX;

        if (probability <= prob_range[0])
            actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                                node_list,edges,weights, boost_graph);
        if (probability <= prob_range[1])
            actions.InterConnections( ik_handler, node_map, node_list,
                                        edges, weights, boost_graph);
        if (probability <= prob_range[2])
            actions.NodeAdditions(ff_frames, ik_handler, wm, geo_filter, node_map,
                    node_list, boost_graph, edges, weights);
        itr ++;
    }
    // actions.InterConnection();
    // actions.GraphSearch();
    // Eigen::VectorXi root_nodes;
    // for (int i=0; i<actions.isCreated[0].rows(); ++i){
    //     for (int j=0; j<8; ++j){
    //         if (actions.isCreated[0](i,j)!=-1){
    //             root_nodes.conservativeResize(root_nodes.size()+1);
    //             root_nodes(root_nodes.size()-1) = actions.isCreated[0](i,j);
    //         }
    //     }
    // }
    node_list[0] = actions.root_nodes;
    ROS_WARN_STREAM( node_list[0].transpose() );

    if (edges.size() > 35000000){ // Safe limit for not blowing up memory
        std::cout<< "\n!!!CAUTION!!!\n";
        std::cout<< "Number of edges exceed the threshold of 35 million.\n";
        std::cout<< "Memory space more than 5 gb will be required.\n";
        std::cout<< "Next safe limit is 50 million with 8 gb memory space.\n";
        std::cout<< "Terminating Planning\n";
        std::cout<< "##############################################################\n\n";
        return false;
    }

    // boost_graph->no_edges = edges.size();
    // boost_graph->no_nodes = node_map.size();
    boost_graph->no_nodes = num_vertices(boost_graph->g);
    boost_graph->no_edges = num_edges(boost_graph->g);
    ROS_WARN_STREAM( boost_graph->no_nodes );
    ROS_WARN_STREAM( boost_graph->no_edges );
    // const int num_nodes = node_map.size();
    // int num_arcs = edges.size();
    // graph_t g(edges.begin(), edges.end(), weights.begin(), num_nodes); boost_graph->g = g;
    std::vector<vertex_descriptor> p(num_vertices(g)); boost_graph->p = p;
    std::vector<double> d(num_vertices(g)); boost_graph->d = d;
    // property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, boost_graph->g);

    boost_graph->no_levels = node_list.size();
    boost_graph->leaf_nodes = node_list[node_list.size()-1];

    std::cout<< "##############################################################\n";
    return true;
};


#endif