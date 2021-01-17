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

double wpCost(std::vector<Eigen::MatrixXi> wps,
                std::vector<node*> node_master_map, std::string type){
    double wp_cost;
    // Analysis Point
    for ( int i=0; i<wps.size()-1; ++i )
        if (wps[i].rows()==0 || wps[i+1].rows()==0)
            return 1e8;

    boost_graph wp_grph;
    graph_t g;
    wp_grph.g = g;
    wp_grph.leaf_connected = true;
    wp_grph.root_connected = true;
    std::vector<Eigen::VectorXi> node_list(wps.size());
    std::vector<Eigen::VectorXd> node_map;
    node_map.push_back(Eigen::VectorXd::Zero(7));
    node_map.push_back(Eigen::VectorXd::Zero(7));
    wp_grph.p.push_back(vertex(0,wp_grph.g));
    wp_grph.p.push_back(vertex(1,wp_grph.g));
    int ctr = 2;

    if (type=="wp"){
        for ( int i=0; i<wps.size(); ++i ){
            for ( int j=0; j<wps[i].rows(); ++j ){
                wp_grph.p.push_back(vertex(ctr,wp_grph.g));
                node_list[i].conservativeResize(node_list[i].size()+1);
                node_list[i](node_list[i].size()-1) = ctr;
                // Obtain the waypoint and push
                node_map.push_back(node_master_map[wps[i](j,0)]->wp_qt);
                ctr++;
            }
        }
    }

    if (type=="q"){
        for ( int i=0; i<wps.size(); ++i ){
            for ( int j=0; j<wps[i].rows(); ++j ){
                for ( int k=0; k<wps[i].cols(); ++k ){
                    if (wps[i](j,k)==-1)
                        continue;
                    wp_grph.p.push_back(vertex(ctr,wp_grph.g));
                    node_list[i].conservativeResize(node_list[i].size()+1);
                    node_list[i](node_list[i].size()-1) = ctr;
                    // Obtain the waypoint and push
                    node_map.push_back(node_master_map[wps[i](j,k)]->jt_config);
                    ctr++;
                }
            }
        }
    }

    // Construct Graph
    for (int i=0; i<node_list.size(); ++i){
        if (i==0)
            for (int j=0; j<node_list[i].size(); ++j)
                add_edge( wp_grph.p[0], wp_grph.p[node_list[i](j)], EdgeWeightProperty(0), wp_grph.g );
        if (i==node_list.size()-1){
            for (int j=0; j<node_list[i].size(); ++j)
                add_edge( wp_grph.p[node_list[i](j)], wp_grph.p[1], EdgeWeightProperty(0), wp_grph.g );
            continue;
        }
        
        for (int j=0; j<node_list[i].size(); ++j){
            for (int k=0; k<node_list[i+1].size(); ++k){
                double cost = (node_map[node_list[i+1](k)]-node_map[node_list[i](j)]).norm();
                add_edge( wp_grph.p[node_list[i](j)], wp_grph.p[node_list[i+1](k)],
                    EdgeWeightProperty(cost), wp_grph.g );
            }
        }
    }
    wp_grph.no_levels = node_list.size();
    std::vector<double> d(num_vertices(wp_grph.g)); wp_grph.d = d;
    Eigen::VectorXi path;
    vertex_descriptor s = vertex(0, wp_grph.g); wp_grph.s = s;
    bool search_success = graph_searches::djikstra(&wp_grph,path,wp_cost);
    return wp_cost;
}

// void RemoveSamples(Actions act, Eigen::VectorXd curr_source, 
//                     Eigen::MatrixXd sources, double radius, std::vector<int>& src_list){
//     Nabo::NNSearchD* tree;
//     Eigen::VectorXi indices(100);
//     Eigen::VectorXd dists2(100);
//     tree = Nabo::NNSearchD::createKDTreeLinearHeap(sources);
//     tree->knn(act.sampler.GetQTWp(curr_source), indices, dists2, indices.size());
//     for (int i=0; i<dists2; ++i){
//         if (dists2(i) <  radius){
//             int index = std::find( src_list.begin(), src_list.end(), indices(i) ) - src_list.begin();
//             if (index < src_list.size()){
//                 vector<int>::iterator it = src_list.begin() + index;
//                 src_list.erase( it );
//             }
//         }
//     else
//         return;
//     }
// }

bool BuildRefineGraph(ikHandler* ik_handler, std::vector<Eigen::MatrixXd>& ff_frames,
                    WM::WM* wm, GeometricFilterHarness* geo_filter,
                    std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                    boost_graph* graph, Eigen::MatrixXd& cost_hist){
    std::cout<< "\n##############################################################\n";
    std::cout<< "Generating Graph\n";
    ik_handler->setTcpFrame(Eigen::MatrixXd::Identity(4,4));
    node_list.resize(ff_frames.size());

    double opt_trigger;
    if(!ros::param::get("/opt_trigger",opt_trigger)){
        std::cout<< "Unable to Obtain Trigger Point\n";
        return 0;
    }

    double alpha;
    if(!ros::param::get("/alpha",alpha)){
        std::cout<< "Unable to Obtain Alpha\n";
        return 0;
    }

    std::cout<< "Number of levels in the graph: " << ff_frames.size() << "\n";
    int itr = 0;
    int max_itr;
    if(!ros::param::get("/max_itr",max_itr)){
        std::cout<< "Unable to Obtain Maximum Itr\n";
        return 0;
    }
    
    ROS_INFO_STREAM("Defining actions object");
    Actions actions(ff_frames);
    ROS_INFO_STREAM("Complete............");
    ROS_INFO_STREAM("Defining graph");
    graph_t g;
    graph->g = g;
    graph->no_levels = ff_frames.size();
    graph->p.push_back(vertex(0,graph->g)); // root node
    graph->p.push_back(vertex(1,graph->g)); // leaf node
    ROS_INFO_STREAM("Complete............");
    ROS_INFO_STREAM("Initializing support variables");
    bool path_found = false;
    double min_cost = 0;
    // Eigen::VectorXd cell_prob[2][2][2][2];
    // InitCellProb(cell_prob);

    Eigen::MatrixXd trajectory(graph->no_levels, ik_handler->OptVarDim);
    Eigen::VectorXi path;
    Eigen::VectorXd path_costs(graph->no_levels-1);
    std::vector<bool> src_bias = {false,false};
    int resource = 1;
    int prev_nodes = 0;
    int tot_nodes = 0;
    bool trigger_random = false;
    double wp_cost = std::numeric_limits<float>::infinity();
    double q_cost = std::numeric_limits<float>::infinity();
    Eigen::MatrixXd fwd_src;
    Eigen::MatrixXd bck_src;
    ROS_INFO_STREAM("Complete............");
    ROS_INFO_STREAM("PCT BEGIN");
    double opt_Cost;
    if(!ros::param::get("/opt_cost",opt_Cost)){
        std::cout<< "Unable to Obtain OptCost\n";
        return 0;
    }

    // Generate Exploration VS Exploitation Profile
    std::vector<int> trigger_itr = { (int)floor(opt_trigger*ff_frames[0].rows()),
                    (int) floor(opt_trigger*ff_frames[graph->no_levels-1].rows()) };
    std::vector<int> domain = { (int)floor((1-opt_trigger)*ff_frames[0].rows()),
                    (int) floor((1-opt_trigger)*ff_frames[graph->no_levels-1].rows()) };
    double x;
    bool fwd_print = true;
    bool bck_print = true;
    ROS_WARN_STREAM("Total number of samples: " << actions.unvisited_src[0].size() + actions.unvisited_src[1].size());
    while(itr < max_itr){
        // int action = GenAction(feature_vec,cell_prob, past_action);

        // Greedy
        x = ff_frames[0].rows()-actions.unvisited_src[0].size();
        if (x > trigger_itr[0]){
            if (fwd_print)
                ROS_WARN_STREAM("******** TRIGGERING FWD BIAS ********");
            fwd_print = false;
            // Probability distribution
            double prob = (double) std::rand() / RAND_MAX;
            // ROS_WARN_STREAM("Sampled Prob: " << prob << ". Current allowed: " << 
            //     exp( -alpha*(x-trigger_itr[0])/(domain[0]) ));
            if( prob < exp( -alpha*(x-trigger_itr[0])/(domain[0]) ) )
                src_bias[0] = false;
            else
                src_bias[0] = true;
        }
        x = ff_frames[graph->no_levels-1].rows()-actions.unvisited_src[1].size();
        if (x > trigger_itr[1]){
            if (bck_print)
                ROS_WARN_STREAM("******** TRIGGERING BCK BIAS ********");
            bck_print = false;
            // Probability distribution
            double prob = (double) std::rand() / RAND_MAX;
            // ROS_WARN_STREAM("Sampled Prob: " << prob << ". Current allowed: " << 
            //     exp( -alpha*(x-trigger_itr[1])/(domain[1]) ));
            if( prob < exp( -alpha*(x-trigger_itr[1])/(domain[1]) ) )
                src_bias[1] = false;
            else
                src_bias[1] = true;
        }
        
        // ROS_INFO_STREAM("Applying Fwd progression");
        if(actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                node_list, graph, "fwd", src_bias[0], resource/2)){
            // q_cost = wpCost(actions.greedy_list, node_map,"wp");
            q_cost = wpCost(actions.greedy_list, node_map,"q");
            if (q_cost < 1e7){
                if (actions.sampler.src_cost[0] > q_cost)
                    actions.sampler.src_cost[0] = q_cost;
                if (actions.sampler.src_cost[1] < q_cost)
                    actions.sampler.src_cost[1] = q_cost;
            }
            // ROS_INFO_STREAM("Fwd costs: Q: " << q_cost <<
            //     ". MinCost: " << actions.sampler.src_cost[0] << ". MaxCost: " << actions.sampler.src_cost[1]);
            actions.sampler.src_costs[0](actions.sampler.src_costs[0].size()-1) = q_cost;
            // if (actions.greedy_list[0].rows() > 0){
            //     int row_id = 0;
            //     while (actions.greedy_list[0](0,row_id)!=-1){row_id++;}
            //     fwd_src.conservativeResize(fwd_src.rows()+1,9);
            //     fwd_src.row(fwd_src.rows()-1) << node_map[actions.greedy_list[0](0,row_id)]->wp_qt.transpose() ,
            //                                 wp_cost, q_cost;
            
            // }
        }
        


        // ROS_INFO_STREAM("Applying Bck progression");
        if(actions.GreedyProgression(ff_frames,ik_handler,wm,geo_filter,node_map,
                node_list, graph, "bck", src_bias[1], resource/2)){
            // q_cost = wpCost(actions.greedy_list, node_map,"wp");
            q_cost = wpCost(actions.greedy_list, node_map,"q");
            if (q_cost < 1e7){
                if (actions.sampler.snk_cost[0] > q_cost)
                    actions.sampler.snk_cost[0] = q_cost;
                if (actions.sampler.snk_cost[1] < q_cost)
                    actions.sampler.snk_cost[1] = q_cost;
            }
            // ROS_INFO_STREAM("Bck costs: Q: " << q_cost <<
            //     ". MinCost: " << actions.sampler.snk_cost[0] << ". MaxCost: " << actions.sampler.snk_cost[1]);
            actions.sampler.src_costs[1](actions.sampler.src_costs[1].size()-1) = q_cost;

            // if (actions.greedy_list[graph->no_levels-1].rows() > 0){
            //     int row_id = 0;
            //     while (actions.greedy_list[graph->no_levels-1](0,row_id)!=-1){row_id++;}
            //     bck_src.conservativeResize(bck_src.rows()+1,9);
            //     bck_src.row(bck_src.rows()-1) << node_map[actions.greedy_list[graph->no_levels-1](0,row_id)]->wp_qt.transpose() ,
            //                                 wp_cost, q_cost;
            
            // }
        }

        // actions.EdgeConnections(ik_handler, node_list, graph, node_map);
        
        // if (trigger_random){
        //     // Random
        //     actions.NodeAdditions(ff_frames, ik_handler, wm, geo_filter, node_map,
        //         node_list, graph, 0.001);
        // }


        // if (src_bias)
        //     return false;

        // if (actions.infeasibility){
        //     ROS_WARN_STREAM("All IKs infeasible at a level");
        //     return false;
        // }
        // double dist_opt = 0;
        // if (path_found){
        //     for (int i=0; i<graph->no_levels-1; ++i)
        //         wp_cost += (node_map[path(i+1)]->wp - node_map[path(i)]->wp).norm();
        //     dist_opt = (node_map[path(0)]->wp-ff_frames[0].row(245).transpose()).norm();
        // }

        graph->no_nodes = num_vertices(graph->g);
        graph->no_edges = num_edges(graph->g);
        std::vector<double> d(num_vertices(graph->g)); graph->d = d;
        if ( GetMinCost(graph, node_map, 
                        ik_handler, min_cost, path_found, path, path_costs, trajectory ) ){}
            // ROS_INFO_STREAM("EOF Iteration: " << itr << ". Path Found");
        else{
            // ROS_INFO_STREAM("EOF Iteration: " << itr << ". Path not found");
        }
        // ROS_WARN_STREAM("G Stats: " << actions.graph_metrics.transpose() << "\n");

        ROS_WARN_STREAM("EOF Iteration: " << itr << ". Path Cost: " 
                        << min_cost << ". # Sources Sampled: " << actions.sampler.src_cnt <<
            ". Nodes Sampled: " << actions.sampler.attempts );

        cost_hist.conservativeResize(3,itr+1);
        cost_hist(0,itr) = min_cost;
        cost_hist(1,itr) = actions.sampler.attempts;
        cost_hist(2,itr) = actions.sampler.src_cnt;


        if (!actions.feasibility){
            ROS_WARN_STREAM("All sources exhausted. Terminating");
            break;
        }

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
    ROS_WARN_STREAM("Cost Profile: " << cost_hist.row(0) << "\n");
    // // Smoothing
    // if (path_found){
    //     ROS_INFO_STREAM("Applying Smoothing");
    //     // Smoothing
    //     for (int i=0; i<graph->no_levels; ++i){
    //         actions.NearestNode( ik_handler, wm, node_map[path(i)]->wp, ff_frames, i,
    //             graph, geo_filter, node_map, node_list, 0.005 );
    //     }
    // }

    // int tot_cnt = 0;
    // int hits = 0;
    // for (int i=0; i<actions.sampler.d_list.size(); ++i){
    //     if (actions.sampler.d_list[i] > 0.5)
    //         hits++;
    //     tot_cnt++;
    // }
    // ROS_WARN_STREAM("Fraction is: " << (double) hits / tot_cnt);
    file_rw::file_write(csv_dir+"../test_case_specific_data/fwd_src_map.csv",fwd_src);
    file_rw::file_write(csv_dir+"../test_case_specific_data/bck_src_map.csv",bck_src);
    // ROS_WARN_STREAM(actions.sampler.src_balls[0].transpose());
    // ROS_WARN_STREAM(actions.sampler.src_balls[1].transpose());
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