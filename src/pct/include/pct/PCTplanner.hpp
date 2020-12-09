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
#include <pct/timer.hpp>
#include <robot_utilities/file_rw.hpp>

int GenAction(Eigen::VectorXd feature_vec, Eigen::VectorXd cell_prob[2][2][2][2], int past_action){
    std::vector<double> x;
    ros::param::get("/decision_var",x);

    // Get Switch Vector
    Eigen::VectorXi switch_vec(4);
    for (int i=0; i<feature_vec.size()-1; ++i)
        if (feature_vec(i) <  x[i])
            switch_vec(i) = 0;
        else
            switch_vec(i) = 1;

    // Get the cell belonging to switch vector and corresponding probabilities
    Eigen::VectorXd probs = cell_prob[switch_vec(0)][switch_vec(1)][switch_vec(2)][switch_vec(3)];
    if (feature_vec(4) < 1e-7){ // Last action was useless so make it's prob zero this time
        probs(past_action) = 0;
        double sum = probs(0) + probs(1) + probs(2) + probs(3);
        probs /= sum;
    }

    double prob = (double) std::rand() / RAND_MAX;
    if (prob <= probs(0))
        return 0;
    if (prob <= probs(0)+probs(1) && prob > probs(0))
        return 1;
    if (prob <= probs(0)+probs(1)+probs(2) && prob > probs(0)+probs(1))
        return 2;
    if (prob <= 1.0 && prob > probs(0)+probs(1)+probs(2))
        return 3;
}

void InitCellProb(Eigen::VectorXd cell_prob[2][2][2][2]){
    std::vector<double> x;
    ros::param::get("/decision_var",x);
    int ctr = 4;
    for (int i=0; i<2; ++i){
        for (int j=0; j<2; ++j){
            for (int k=0; k<2; ++k){
                for (int l=0; l<2; ++l){
                    cell_prob[i][j][k][l].resize(4);
                    double sum = x[ctr] + x[ctr+1] + x[ctr+2] + x[ctr+3];
                    cell_prob[i][j][k][l](0) = x[ctr]/sum; ctr++;
                    cell_prob[i][j][k][l](1) = x[ctr]/sum; ctr++;
                    cell_prob[i][j][k][l](2) = x[ctr]/sum; ctr++;
                    cell_prob[i][j][k][l](3) = x[ctr]/sum; ctr++;
                }
            }
        }
    }
    return;
}

Eigen::VectorXd GetFeatureVector(Eigen::VectorXd graph_metrics, int no_levels, 
                double cost_change, Eigen::VectorXd past_feature){
    Eigen::VectorXd feature_vec(5);
    feature_vec(0) = 0.33*graph_metrics(5)/graph_metrics(4) // Valid/Total Fwd sources
                        + 0.33*graph_metrics(6)/no_levels // Avg fwd depth/Total depth
                        + 0.33*graph_metrics(7)/no_levels; // Max fwd depth/Total depth
    feature_vec(1) = 0.33*graph_metrics(10)/graph_metrics(9) // Valid/Total Bck sources
                        + 0.33*graph_metrics(11)/no_levels // Avg bck depth/Total depth
                        + 0.33*graph_metrics(12)/no_levels; // Max bck depth/Total depth
    feature_vec(2) = graph_metrics(1)/graph_metrics(0); // Edges/Total edges
    feature_vec(3) = graph_metrics(3)/graph_metrics(2); // Nodes/Total nodes
    feature_vec(4) = (feature_vec.segment(0,4)-past_feature.segment(0,4)).norm();
    return feature_vec;
};

std::string csv_dir = ros::package::getPath("pct") + "/data/csv/";
// int file_id = 0;

bool GetMinCost(boost_graph* g, Eigen::VectorXi root_nodes,
                std::vector<node*>& node_map, ikHandler* ik_handler,
                   double& min_cost, bool& path_found, Eigen::VectorXi &path,
                    std::vector<double>& path_costs,
                   Eigen::MatrixXd& trajectory ){
    boost_graph graph = *g;
    path.resize(graph.no_levels);
    if (graph.leaf_nodes.size()==0)
        return false;
    double lowest_cost = std::numeric_limits<double>::infinity();
    trajectory.resize(graph.no_levels, ik_handler->robot->NrOfJoints);
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
                path_costs[k+1] = cost;
            }
            if (path_cost<lowest_cost){
                lowest_cost = path_cost;
                trajectory = curr_traj;
                path = id_path;
            }
        }
    }
    // // Saving trajectory
    // file_rw::file_write(csv_dir+std::to_string(file_id)+".csv",trajectory);
    // file_id++;
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
    int max_time;
    if(!ros::param::get("/max_pct_time",max_time)){
        std::cout<< "Unable to Obtain Maximum Iterations\n";
        return 0;
    }
    
    Actions actions(ff_frames);
    graph_t g;
    boost_graph->g = g;
    boost_graph->no_levels = node_list.size();

    // int no_interconnections = (int) 0.2*node_list.size();
    // int no_node_insertions = (int) 0.2*node_list.size();
    int no_interconnections = 1;
    int no_node_insertions = 1;
    bool path_found = false;
    double min_cost = 0;
    double prev_cost = 0;
    std::vector<double> path_costs(ff_frames.size());
    for (int i=0; i<path_costs.size(); ++i)
        path_costs[i] = 0;
    Eigen::VectorXd cell_prob[2][2][2][2];
    InitCellProb(cell_prob);

    Eigen::VectorXd feature_vec = Eigen::VectorXd::Zero(5);
    feature_vec(4) = std::numeric_limits<double>::infinity();
    int past_action = 0;
    Eigen::VectorXd past_feature = Eigen::VectorXd::Zero(5);
    past_feature(4) = 0;

    timer main_timer;
    main_timer.start();
    // while ( itr<max_time || actions.root_nodes.size()==0 || !path_found ){
    // while ( main_timer.elapsed()<max_time ){
    std::vector<int> fractions(10); // Fraction for which greedy progression to be done
    // fractions = {0,0,0,0,0,0,0,0,0,0};
    fractions = {10,10,10,10};

    Eigen::VectorXd cost_hist(fractions.size());
    Eigen::MatrixXd cost_vec(fractions.size(),boost_graph->no_levels);
    Eigen::MatrixXd cost_grad;
    cost_grad = Eigen::MatrixXd::Zero(boost_graph->no_levels,2);
    Eigen::VectorXd prev_cost_vec;
    Eigen::MatrixXd trajectory;
    Eigen::VectorXi path;
    while(itr<fractions.size()){
        int action = GenAction(feature_vec,cell_prob, past_action);
        past_action = action;
        // if (action==0 || itr==0){
        // if (itr==0){
        //     ROS_INFO_STREAM("Performing Initialization");

        for (int i=0; i<fractions[itr]/2; ++i){
            actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                            node_list,edges,weights, boost_graph, "fwd");
            actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                            node_list,edges,weights, boost_graph, "bck");
            if (actions.infeasibility){
                ROS_WARN_STREAM("All IKs infeasible at a level");
                return false;
            }
        }
        //     ROS_INFO_STREAM("Performing Edge Connections");
        //     actions.EdgeConnections(ik_handler, node_list, boost_graph, 
        //                             edges, weights, node_map);
        //     ROS_WARN_STREAM("EOF Iteration: " << itr << ". Initialization Complete\n");
        // }
        // if (action==1 || itr==0){
        //     ROS_INFO_STREAM("Action-2: Backward Progression");
        //     actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
        //                         node_list,edges,weights, boost_graph, "bck");
        // }
        
        // if (action==2 || itr==0){
        //     ROS_INFO_STREAM("Action-3: InterConnections");
        //     for (int i=0; i<no_interconnections; ++i)
        //         actions.InterConnections( ik_handler, node_map, node_list,
        //                                 edges, weights, boost_graph, min_cost,path_costs);
        // }
        // if (action==3 || itr==0){
            // ROS_INFO_STREAM("Node Insertion");

        for (int i=0; i<10-fractions[itr]; ++i){
            actions.NodeAdditions(ff_frames, ik_handler, wm, geo_filter, node_map,
                node_list, boost_graph, edges, weights, min_cost,path_costs, cost_grad);
        }
        
        actions.EdgeConnections(ik_handler, node_list, boost_graph, 
                                    edges, weights, node_map);

        if (actions.infeasibility){
            ROS_WARN_STREAM("All IKs infeasible at a level");
            return false;
        }

        boost_graph->leaf_nodes = node_list[node_list.size()-1];
        boost_graph->no_nodes = num_vertices(boost_graph->g);
        boost_graph->no_edges = num_edges(boost_graph->g);
        std::vector<double> d(num_vertices(boost_graph->g)); boost_graph->d = d;

        if ( GetMinCost(boost_graph, actions.root_nodes, node_map, 
                        ik_handler, min_cost, path_found, path, path_costs, trajectory ) )
            ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path Found");
        else
            ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path not found");
        // ROS_WARN_STREAM("G Stats: " << actions.graph_metrics.transpose() << "\n");
        ROS_INFO_STREAM("Path Cost: " << min_cost);
        feature_vec = 
                    GetFeatureVector(actions.graph_metrics, boost_graph->no_levels, 
                                    0, past_feature );
        past_feature = feature_vec;
        ROS_WARN_STREAM("Feature Vector: " << feature_vec.transpose() << "\n");
        prev_cost = min_cost;

        // // Update Connection Costs Vector
        // ROS_WARN_STREAM(cost_vec.transpose());
        cost_hist(itr) = min_cost;
        double* ptr = &path_costs[0];
        Eigen::Map<Eigen::VectorXd> curr_cost_vec(ptr, path_costs.size());
        cost_vec.row(itr) = curr_cost_vec.transpose();

        // Compute gradient of cost vector
        for (int i=0; i<curr_cost_vec.size(); ++i){
            double sum = 0;
            int denom = 0;
            if (i-1>=0){
                sum += std::fabs(curr_cost_vec(i)-curr_cost_vec(i-1));
                denom++;
            }
            if (i+1<curr_cost_vec.size()){
                sum += std::fabs(curr_cost_vec(i)-curr_cost_vec(i+1));
                denom++;
            }
            cost_grad(i,0) = sum / denom;
        }
        if (prev_cost_vec.size()>0)
            cost_grad.col(1) = (curr_cost_vec - prev_cost_vec).array().abs();
        prev_cost_vec = curr_cost_vec;        
        itr ++;
    }
    
    ROS_WARN_STREAM("Initiating Path Smoothing based on Graph Gradients");
    Eigen::MatrixXd diff_mat(64,6);
    int cnt = 0;
    double delta = 0.08;
    for (int i=0; i<2; ++i){
        for (int j=0; j<2; ++j){
            for (int k=0; k<2; ++k){
                for (int l=0; l<2; ++l){
                    for (int m=0; m<2; ++m){
                        for (int n=0; n<2; ++n){
                            diff_mat.row(cnt) << i,j,k,l,m,n;
                            diff_mat.row(cnt) *= delta;
                            cnt++;
                        }
                    }
                }
            }
        }
    }

    Eigen::VectorXi neigh_insert = Eigen::VectorXi::Ones(boost_graph->no_levels);
    Eigen::MatrixXd dir_vecs(boost_graph->no_levels,6);
    ROS_WARN_STREAM( "No nodes before: " << boost_graph->no_nodes );
    ROS_WARN_STREAM( "No edges before: " << boost_graph->no_edges );

    int max_itr = 10;
    // Eigen::VectorXd hist(samples);
    while (itr < max_itr){
        for (int i=0; i<path.size(); ++i){
            actions.NearestNode(ik_handler, wm, node_map[path(i)]->wp,
                ff_frames, i, boost_graph, geo_filter,
                node_map, node_list);
        }

        // std::vector<Eigen::MatrixXd> new_nodes(boost_graph->no_levels);
        // Determine nodes
        // for (int i=0; i<neigh_insert.size(); ++i){
        //     if(neigh_insert(i)==0){
        //         Eigen::VectorXd jt_config = (trajectory.row(i) + dir_vecs.row(i)*delta).transpose();
        //         actions.NearestNode(ik_handler, wm, jt_config,
        //         ff_frames, i, boost_graph, geo_filter,
        //         node_map, node_list);
        //         continue;
        //     }

        //     for (int j=0; j<64; ++j){
        //         Eigen::VectorXd jt_config = (trajectory.row(i) + diff_mat.row(j)).transpose();
        //         actions.NearestNode(ik_handler, wm, jt_config,
        //                         ff_frames, i, boost_graph, geo_filter,
        //                         node_map, node_list);
        //     }
        // }

        // // Insert Nodes
        // for (int i=0; i<new_nodes.size(); ++i){
        //     for (int j=0; j<new_nodes[i].rows(); ++j){
        //         int node_id = node_map.size();
        //         node* new_node = new node;
        //         new_node->id = node_id;
        //         new_node->jt_config = new_nodes[i].row(j).transpose();
        //         new_node->depth = i;
        //         node_map.push_back(new_node);
        //         node_list[i].conservativeResize(node_list[i].size()+1);
        //         node_list[i](node_list[i].size()-1) = node_id;
        //         boost_graph->p.push_back(vertex(node_id,boost_graph->g));
        //     }
        // }

        actions.EdgeConnections(ik_handler, node_list, boost_graph, 
                                edges, weights, node_map);

        boost_graph->leaf_nodes = node_list[node_list.size()-1];
        boost_graph->no_nodes = num_vertices(boost_graph->g);
        boost_graph->no_edges = num_edges(boost_graph->g);
        std::vector<double> d(num_vertices(boost_graph->g)); boost_graph->d = d;
        Eigen::MatrixXd traj;
        if ( GetMinCost(boost_graph, actions.root_nodes, node_map, 
                        ik_handler, min_cost, path_found, path, path_costs, traj ) )
            ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path Found");
        else
            ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path not found");
        ROS_INFO_STREAM("Path Cost: " << min_cost << "\n");
        // hist(i-1) = min_cost;
        dir_vecs = traj - trajectory;
        for (int i=0; i<dir_vecs.rows(); ++i){
            if (dir_vecs.row(i).norm() < 1e-5)
                neigh_insert(i) = 1;
            else{
                neigh_insert(i) = 0;
                dir_vecs.row(i) /= dir_vecs.row(i).norm();
            }
        }
        trajectory = traj;
        itr++;        
    }

    // std::cout<< hist.transpose() << "\n";

    // Eigen::MatrixXd zbc = cost_vec.transpose();
    // file_rw::file_write(csv_dir+"cost_vecs.csv",zbc);
    // ROS_WARN_STREAM("Cost history: " << cost_vec.transpose());

    // std::fabs(min_cost-prev_cost) / prev_cost
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
    ROS_WARN_STREAM( "No nodes after: " << boost_graph->no_nodes );
    ROS_WARN_STREAM( "No edges after: " << boost_graph->no_edges );
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





// std::string csv_dir;
// // csv_dir = ros::package::getPath("pct") + "/data/csv/";

// Eigen::MatrixXd opt_configs = file_rw::file_read_mat(
// csv_dir+"../test_case_specific_data/gear/opt_states.csv");
// int samples = 10;
// Eigen::MatrixXd jt_diff = (opt_configs - trajectory) / samples;
// Eigen::VectorXd hist(samples);

// for (int i=1; i<=samples; ++i){
//     Eigen::MatrixXd new_nodes = trajectory + jt_diff * i;
//     for (int j=0; j<new_nodes.rows(); ++j){
//         int node_id = node_map.size();
//         node* new_node = new node;
//         new_node->id = node_id;
//         new_node->jt_config = new_nodes.row(j).transpose();
//         new_node->depth = j;
//         node_map.push_back(new_node);
//         node_list[j].conservativeResize(node_list[j].size()+1);
//         node_list[j](node_list[j].size()-1) = node_id;
//         boost_graph->p.push_back(vertex(node_id,boost_graph->g));
//     }

//     actions.EdgeConnections(ik_handler, node_list, boost_graph, 
//                                 edges, weights, node_map);

//     boost_graph->leaf_nodes = node_list[node_list.size()-1];
//     boost_graph->no_nodes = num_vertices(boost_graph->g);
//     boost_graph->no_edges = num_edges(boost_graph->g);
//     std::vector<double> d(num_vertices(boost_graph->g)); boost_graph->d = d;
//     Eigen::MatrixXd traj;
//     if ( GetMinCost(boost_graph, actions.root_nodes, node_map, 
//                     ik_handler, min_cost, path_found, path_costs, traj ) )
//         ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path Found");
//     else
//         ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path not found");
//     // ROS_WARN_STREAM("G Stats: " << actions.graph_metrics.transpose() << "\n");
//     ROS_INFO_STREAM("Path Cost: " << min_cost);
//     hist(i-1) = min_cost;
//     itr++;
// }