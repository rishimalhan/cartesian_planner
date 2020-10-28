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

Eigen::VectorXd GetFeatureVector(Eigen::VectorXd graph_metrics, int no_levels){
    Eigen::VectorXd feature_vec(8);
    feature_vec(0) = graph_metrics(5)/graph_metrics(4); // Valid/Total Fwd sources
    feature_vec(1) = graph_metrics(10)/graph_metrics(9); // Valid/Total Bck sources
    feature_vec(2) = graph_metrics(6)/no_levels; // Avg fwd depth/Total depth
    feature_vec(3) = graph_metrics(7)/no_levels; // Max fwd depth/Total depth
    feature_vec(4) = graph_metrics(11)/no_levels; // Avg bck depth/Total depth
    feature_vec(5) = graph_metrics(12)/no_levels; // Max bck depth/Total depth
    feature_vec(6) = graph_metrics(1)/graph_metrics(0); // Edges/Total edges
    feature_vec(7) = graph_metrics(3)/graph_metrics(2); // Nodes/Total nodes
    return feature_vec;
};

bool GetMinCost(boost_graph* g, Eigen::VectorXi root_nodes,
                std::vector<node*>& node_map, ikHandler* ik_handler,
                   double& min_cost, bool& path_found, std::vector<double>& path_costs ){
    boost_graph graph = *g;
    if (graph.leaf_nodes.size()==0)
        return false;
    double lowest_cost = std::numeric_limits<double>::infinity();
    // Run Search For Multiple Start Configs
    for (int i=0; i<root_nodes.size(); ++i){
        int root_node = root_nodes(i);
        // Change the start vertex in graph
        vertex_descriptor s = vertex(root_node, graph.g); graph.s = s;
        Eigen::VectorXi id_path;
        bool search_success = graph_searches::djikstra(&graph,id_path);
        if(search_success){ // Get the shortest path to a leaf node in terms of node ids
            path_found = true;
            // Generate Trajectory
            Eigen::MatrixXd curr_traj(graph.no_levels, ik_handler->robot->NrOfJoints);
            for(int k=0; k<id_path.size(); ++k)
                curr_traj.row(k) = node_map[id_path(k)]->jt_config.transpose();
            // Evaluate trajectory cost
            double path_cost = 0;
            for (int k=0; k<curr_traj.rows()-1;++k){
                Eigen::ArrayXd jt_diff = (curr_traj.row(k+1) - curr_traj.row(k)).transpose();
                double cost = jt_diff.abs().maxCoeff();
                path_cost += cost;
                path_costs[k] = cost;
            }
            if (path_cost<lowest_cost)
                lowest_cost = path_cost;
        }
    }
    min_cost = lowest_cost;
    return path_found;
};


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
    boost_graph->no_levels = node_list.size();

    // Determine Conditions and assign probability to actions
    std::vector<double> prob_range(4);
    prob_range[0] = 0.2;
    prob_range[1] = 0.2;
    prob_range[2] = 0.8;
    prob_range[3] = 0.8;
    bool path_found = false;
    double min_cost;
    std::vector<double> path_costs(ff_frames.size());
    for (int i=0; i<path_costs.size(); ++i)
        path_costs[i] = 0;
    // Take action
    while ( itr<max_itr || actions.root_nodes.size()==0 || !path_found ){
        // Sample from distribution
        double probability = (double) std::rand() / RAND_MAX;
        if (itr==0)
            probability = 0.0;
        if (probability <= prob_range[0]){
            ROS_INFO_STREAM("Action-1: Forward Progression");
            actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                                node_list,edges,weights, boost_graph, "fwd");
        }
        if (probability <= prob_range[1]){
            ROS_INFO_STREAM("Action-2: Backward Progression");
            actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                                node_list,edges,weights, boost_graph, "bck");
        }
        if (probability <= prob_range[2]){
            ROS_INFO_STREAM("Action-3: InterConnections");
            actions.InterConnections( ik_handler, node_map, node_list,
                                        edges, weights, boost_graph, path_costs);
        }
        if (probability <= prob_range[3]){
            ROS_INFO_STREAM("Action-4: Node Insertion");
            actions.NodeAdditions(ff_frames, ik_handler, wm, geo_filter, node_map,
                    node_list, boost_graph, edges, weights);
        }
        itr ++;
        
        if (actions.infeasibility){
            ROS_WARN_STREAM("All IKs infeasible at a level");
            return false;
        }

        boost_graph->leaf_nodes = node_list[node_list.size()-1];
        boost_graph->no_nodes = num_vertices(boost_graph->g);
        boost_graph->no_edges = num_edges(boost_graph->g);
        std::vector<double> d(num_vertices(boost_graph->g)); boost_graph->d = d;

        if ( GetMinCost(boost_graph, actions.root_nodes, node_map, 
                        ik_handler, min_cost, path_found, path_costs ) )
            ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path Found");
        else
            ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path not found");
        // ROS_WARN_STREAM("G Stats: " << actions.graph_metrics.transpose() << "\n");
        ROS_INFO_STREAM("Path Cost: " << min_cost);
        Eigen::VectorXd feature_vec = 
                    GetFeatureVector(actions.graph_metrics, boost_graph->no_levels);
        ROS_WARN_STREAM("Feature Vector: " << feature_vec.transpose() << "\n");

    }
    if (!path_found)
        return false;
    node_list[0] = actions.root_nodes;

    if (edges.size() > 35000000){ // Safe limit for not blowing up memory
        std::cout<< "\n!!!CAUTION!!!\n";
        std::cout<< "Number of edges exceed the threshold of 35 million.\n";
        std::cout<< "Memory space more than 5 gb will be required.\n";
        std::cout<< "Next safe limit is 50 million with 8 gb memory space.\n";
        std::cout<< "Terminating Planning\n";
        std::cout<< "##############################################################\n\n";
        return false;
    }

    boost_graph->no_nodes = num_vertices(boost_graph->g);
    boost_graph->no_edges = num_edges(boost_graph->g);
    // ROS_WARN_STREAM( "No nodes in graph: " << boost_graph->no_nodes );
    // ROS_WARN_STREAM( "No edges in graph: " << boost_graph->no_edges );
    // const int num_nodes = node_map.size();
    // int num_arcs = edges.size();
    // graph_t g(edges.begin(), edges.end(), weights.begin(), num_nodes); boost_graph->g = g;
    // std::vector<vertex_descriptor> p(num_vertices(g)); boost_graph->p = p;
    std::vector<double> d(num_vertices(boost_graph->g)); boost_graph->d = d;
    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, boost_graph->g);

    boost_graph->leaf_nodes = node_list[node_list.size()-1];

    std::cout<< "##############################################################\n";
    return true;
};


#endif