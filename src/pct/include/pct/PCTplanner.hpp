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

bool GetMinCost(boost_graph* g, std::vector<node*>& node_map, ikHandler* ik_handler,
                   double& min_cost, bool& path_found, Eigen::VectorXi &path,
                    Eigen::VectorXd& path_costs,
                   Eigen::MatrixXd& trajectory ){
    boost_graph graph = *g;
    vertex_descriptor s = vertex(0, graph.g); graph.s = s;
    bool search_success = graph_searches::djikstra(&graph,path,min_cost);
    if(search_success){ // Get the shortest path to a leaf node in terms of node ids
        path_found = true;
        // Generate Trajectory
        for(int k=0; k<path.size(); ++k)
            trajectory.row(k) = node_map[path(k)]->jt_config.transpose();
        for (int k=0; k<trajectory.rows()-1;++k)
            path_costs(k) = (trajectory.row(k+1) - trajectory.row(k)).norm();
    }
    return path_found;
};

double wpCost(std::vector<Eigen::MatrixXd> wps){
    double wp_cost = 0;
    // Analysis Point
    for ( int i=0; i<wps.size()-1; ++i ){
        if (wps[i].cols()==0 || wps[i+1].cols()==0)
            return 6.5;
        Eigen::VectorXd curr_wp = wps[i].rowwise().mean();
        double sum = 0;
        for (int j=0; j<wps[i+1].cols(); ++j)
            sum += (wps[i+1].col(j) - curr_wp).norm();
        wp_cost += sum / wps[i+1].cols();
    }
    return wp_cost;
}

bool BuildRefineGraph(ikHandler* ik_handler, std::vector<Eigen::MatrixXd>& ff_frames,
                    WM::WM* wm, GeometricFilterHarness* geo_filter,
                    std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                    boost_graph* graph, Eigen::MatrixXd& cost_hist){
    std::cout<< "\n##############################################################\n";
    std::cout<< "Generating Graph\n";
    ik_handler->setTcpFrame(Eigen::MatrixXd::Identity(4,4));
    node_list.resize(ff_frames.size());

    std::cout<< "Number of levels in the graph: " << ff_frames.size() << "\n";
    int itr = 0;
    int max_time;
    if(!ros::param::get("/max_pct_time",max_time)){
        std::cout<< "Unable to Obtain Maximum Iterations\n";
        return 0;
    }
    
    // ROS_INFO_STREAM("Defining actions object");
    Actions actions(ff_frames);
    // ROS_INFO_STREAM("Complete............");
    // ROS_INFO_STREAM("Defining graph");
    graph_t g;
    graph->g = g;
    graph->no_levels = ff_frames.size();
    graph->p.push_back(vertex(0,graph->g)); // root node
    graph->p.push_back(vertex(1,graph->g)); // leaf node
    // ROS_INFO_STREAM("Complete............");
    // ROS_INFO_STREAM("Initializing support variables");
    bool path_found = false;
    double min_cost;
    // Eigen::VectorXd cell_prob[2][2][2][2];
    // InitCellProb(cell_prob);

    // Eigen::VectorXd feature_vec = Eigen::VectorXd::Zero(5);
    // feature_vec(4) = std::numeric_limits<double>::infinity();
    // int past_action = 0;
    // Eigen::VectorXd past_feature = Eigen::VectorXd::Zero(5);
    // past_feature(4) = 0;

    // timer main_timer;
    // main_timer.start();
    // while ( itr<max_time || actions.root_nodes.size()==0 || !path_found ){
    // while ( main_timer.elapsed()<max_time ){
    std::vector<int> fractions; // Fraction for which greedy progression to be done
    fractions = {10,10,10,10,10,10,10,10,10,10,
                    10,10,10,10,10,10,10,10,10,10,
                    10,10,10,10,10,10,10,10,10,10,
                    10,10,10,10,10,10,10,10,10,10,
                    10,10,10,10,10,10,10,10,10,10,
                    10,10,10,10,10,10,10,10,10,10,
                    10,10,10,10,10,10,10,10,10,10}; // Greedy

    // fractions = {10,10,10,10,10,8,8,8,8,8,
    //                 6,6,6,6,4,4,4,4,2,2,
    //                 2,0,0,0,0,0,0,0,0,0,
    //                 0,0,0,0,0,0,0,0,0,0,
    //                 0,0,0,0,0,0,0,0,0,0,
    //                 0,0,0,0,0,0,0,0,0,0,
    //                 0,0,0,0,0,0,0,0,0,0}; // Greedy + Random

    // fractions = {10,10,10,6,4,4,2,0,0,0}; // Greedy + Smoothing or Random + Smoothing
    // fractions = {10,8,6,4,2,0,0,0,0,0}; // Greedy + Random


    // fractions = {0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 
    //             0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,
    //             0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,
    //             0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,
    //             0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,
    //             0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0 };

    Eigen::MatrixXd trajectory(graph->no_levels, ik_handler->OptVarDim);
    Eigen::VectorXi path;
    cost_hist.conservativeResize(3,fractions.size());
    cost_hist.row(1) = Eigen::VectorXd::Zero(fractions.size()).transpose();
    Eigen::VectorXd path_costs(graph->no_levels-1);
    bool diversity = false;
    int resource = 1;
    int prev_nodes = 0;
    int tot_nodes = 0;
    // ROS_INFO_STREAM("Complete............");
    // ROS_INFO_STREAM("PCT BEGIN");
    while(itr < fractions.size()){
        // int action = GenAction(feature_vec,cell_prob, past_action);

        // Greedy
        if ( path_found && (actions.sampler.src_waypoints.cols()>3 && actions.sampler.snk_waypoints.cols()>3) )
            diversity = true;
        
        ROS_INFO_STREAM("Applying Fwd/Bck progression");
        actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                node_list, graph, "fwd", diversity, resource/2);
        actions.sampler.src_balls[0](actions.sampler.src_balls[0].size()-1) = 
                                                (wpCost(actions.greedy_list)-3.0) / (6.5-3.0) * actions.delta;
        actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                node_list, graph, "bck", diversity, resource/2);
        actions.sampler.src_balls[1](actions.sampler.src_balls[1].size()-1) = 
                                                (wpCost(actions.greedy_list)-3.0) / (6.5-3.0) * actions.delta;

        // Random
        // actions.NodeAdditions(ff_frames, ik_handler, wm, geo_filter, node_map,
        //     node_list, graph, 10-fractions[itr]);
        

        // Smoothing
        if (path_found){
            ROS_INFO_STREAM("Applying Smoothing");
            // Smoothing
            for (int i=0; i<graph->no_levels; ++i){
                actions.NearestNode( ik_handler, wm, node_map[path(i)]->wp, ff_frames, i,
                    graph, geo_filter, node_map, node_list, 10 );
            }
        }

        actions.EdgeConnections(ik_handler, node_list, graph, node_map);

        // if (actions.infeasibility){
        //     ROS_WARN_STREAM("All IKs infeasible at a level");
        //     return false;
        // }
        double dist_opt = 0;
        // if (path_found){
        //     for (int i=0; i<graph->no_levels-1; ++i)
        //         wp_cost += (node_map[path(i+1)]->wp - node_map[path(i)]->wp).norm();
        //     dist_opt = (node_map[path(0)]->wp-ff_frames[0].row(245).transpose()).norm();
        // }


        graph->no_nodes = num_vertices(graph->g);
        graph->no_edges = num_edges(graph->g);
        std::vector<double> d(num_vertices(graph->g)); graph->d = d;
        if ( GetMinCost(graph, node_map, 
                        ik_handler, min_cost, path_found, path, path_costs, trajectory ) )
            ROS_INFO_STREAM("EOF Iteration: " << itr << ". Path Found");
        else
            ROS_INFO_STREAM("EOF Iteration: " << itr << ". Path not found");
        // ROS_WARN_STREAM("G Stats: " << actions.graph_metrics.transpose() << "\n");


        ROS_WARN_STREAM("Path Cost: " << min_cost << ". Workspace Cost: " << wpCost(actions.greedy_list) <<
                            ". Dist to opt: " << dist_opt );
        cost_hist(0,itr) = min_cost;
        // int no_nodes = num_vertices(graph->g);
        // tot_nodes += no_nodes - prev_nodes;
        cost_hist(1,itr) = actions.sampler.attempts;
        cost_hist(2,itr) = actions.sampler.src_cnt;


        // prev_nodes = no_nodes;

        // ROS_WARN_STREAM("path" << path.transpose());
        // feature_vec = 
        //             GetFeatureVector(actions.graph_metrics, graph->no_levels, 
        //                             0, past_feature );
        // past_feature = feature_vec;
        // ROS_WARN_STREAM("Feature Vector: " << feature_vec.transpose() << "\n");


        // Eigen::MatrixXd points(path.size(),12);
        // for (int i=0; i<path.size(); ++i){
        //     points.block(i,0,1,3) = node_map[path(i)]->wp.segment(0,3).transpose();
        //     // points.block(i,3,1,3) = rtf::bxbybz2eul(node_map[path(i)]->
        //     //                         wp.segment(3,9).transpose(),
        //     //                         "ZYX");
        //     points.block(i,3,1,9) = node_map[path(i)]->wp.segment(3,9).transpose();
        // }

        // file_rw::file_write(csv_dir+"../test_case_specific_data/configs"+std::to_string(itr)+".csv",
        //                         trajectory);
        // file_rw::file_write(csv_dir+"../test_case_specific_data/points"+std::to_string(itr)+".csv",
        //                         points);

        itr ++;
    }
    // for (int i=0; i<graph->no_levels; ++i)
    //     delete actions.sampler.kdtrees[i];
    // actions.sampler.kdtrees.clear();


    // Eigen::MatrixXd a = actions.sampler.src_waypoints.transpose();
    // Eigen::MatrixXd b = actions.sampler.snk_waypoints.transpose();
    // file_rw::file_write(csv_dir+"../test_case_specific_data/source.csv", a);
    // file_rw::file_write(csv_dir+"../test_case_specific_data/sink.csv", b);

    // for ( int i=0; i<node_list.size(); ++i ){
    //     Eigen::MatrixXd configs(node_list[i].size(),6);
    //     Eigen::MatrixXd wps(node_list[i].size(),12);
    //     for (int j=0; j<node_list[i].size(); ++j){
    //         configs.row(j) = node_map[node_list[i](j)]->jt_config.transpose();
    //         wps.row(j) = node_map[node_list[i](j)]->wp.transpose();
    //     }
    //     file_rw::file_write(csv_dir+"../test_case_specific_data/wp"+std::to_string(i)+".csv",
    //                             wps);
    //     file_rw::file_write(csv_dir+"../test_case_specific_data/"+std::to_string(i)+".csv",
    //                             configs);
    // }


    if (!path_found)
        return false;

    if (graph->no_edges > 35000000){ // Safe limit for not blowing up memory
        std::cout<< "\n!!!CAUTION!!!\n";
        std::cout<< "Number of edges exceed the threshold of 35 million.\n";
        std::cout<< "Memory space more than 5 gb will be required.\n";
        std::cout<< "Next safe limit is 50 million with 8 gb memory space.\n";
        std::cout<< "Terminating Planning\n";
        std::cout<< "##############################################################\n\n";
        return false;
    }

    graph->no_nodes = num_vertices(graph->g);
    graph->no_edges = num_edges(graph->g);
    ROS_WARN_STREAM( "No nodes in graph: " << graph->no_nodes );
    ROS_WARN_STREAM( "No edges in graph: " << graph->no_edges );
    std::vector<double> d(num_vertices(graph->g)); graph->d = d;
    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, graph->g);

    std::cout<< "##############################################################\n";
    return true;
};


#endif





// std::vector<Eigen::MatrixXd> new_nodes(graph->no_levels);
// Determine nodes
// for (int i=0; i<neigh_insert.size(); ++i){
    // if(neigh_insert(i)==0){
    //     Eigen::VectorXd jt_config = (trajectory.row(i) + dir_vecs.row(i)*delta).transpose();
    //     actions.NearestNode(ik_handler, wm, jt_config,
    //     ff_frames, i, graph, geo_filter,
    //     node_map, node_list);
    //     continue;
    // }

//     for (int j=0; j<64; ++j){
//         Eigen::VectorXd jt_config = (trajectory.row(i) + diff_mat.row(j)).transpose();
//         actions.NearestNode(ik_handler, wm, jt_config,
//                         ff_frames, i, graph, geo_filter,
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
//         graph->p.push_back(vertex(node_id,graph->g));
//     }
// }



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
//         graph->p.push_back(vertex(node_id,graph->g));
//     }

//     actions.EdgeConnections(ik_handler, node_list, graph, 
//                                 edges, weights, node_map);

//     graph->leaf_nodes = node_list[node_list.size()-1];
//     graph->no_nodes = num_vertices(graph->g);
//     graph->no_edges = num_edges(graph->g);
//     std::vector<double> d(num_vertices(graph->g)); graph->d = d;
//     Eigen::MatrixXd traj;
//     if ( GetMinCost(graph, actions.root_nodes, node_map, 
//                     ik_handler, min_cost, path_found, path_costs, traj ) )
//         ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path Found");
//     else
//         ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path not found");
//     // ROS_WARN_STREAM("G Stats: " << actions.graph_metrics.transpose() << "\n");
//     ROS_INFO_STREAM("Path Cost: " << min_cost);
//     hist(i-1) = min_cost;
//     itr++;
// }