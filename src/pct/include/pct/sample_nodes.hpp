#ifndef __SAMPLENODES__HPP__
#define __SAMPLENODES__HPP__

#include <pct/node_description.hpp>
#include <random>
#include <Eigen/Eigen>
#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/transformation_utilities.hpp>
#include <pct/geometric_filter.h>
#include <pct/graph_description.hpp>

typedef std::priority_queue<std::pair<int,double>> CostQ;

class FFSampler{
private:
int prev_index;
int prev_depth;

bool isEdge(const std::vector<node*>& node_map, const int parent, const int child){
    if ((node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff() > 1.57)
        return false;
    return true;
};

double computeGCost( const std::vector<node*>& node_map, const int parent, const int child ){
    // return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
    // return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
    return (node_map[child]->jt_config - node_map[parent]->jt_config).norm();
};

bool MakeConnections( Eigen::VectorXi parents, Eigen::VectorXi children, 
                    ikHandler* ik_handler, const std::vector<node*>& node_map, boost_graph* boost_graph){
        bool atleast_one_edge = false;
        if (parents.size()==0 || children.size()==0)
            return atleast_one_edge;

        // Connect dummy root
        if (node_map[parents(0)]->depth==0){
            boost_graph->root_connected = true;
            for (int i=0; i<parents.size(); ++i)
                add_edge( boost_graph->p[0], boost_graph->p[parents(i)],
                            EdgeWeightProperty(0),boost_graph->g );
        }
        if (node_map[parents(0)]->depth==boost_graph->no_levels-1){
            boost_graph->root_connected = true;
            for (int i=0; i<parents.size(); ++i)
                add_edge( boost_graph->p[parents(i)], boost_graph->p[1],
                            EdgeWeightProperty(0),boost_graph->g );
        }
        
        // Connect dummy leaf
        if (node_map[children(0)]->depth==boost_graph->no_levels-1){
            boost_graph->leaf_connected = true;
            for (int j=0; j<children.size(); ++j)
                add_edge( boost_graph->p[children(j)], boost_graph->p[1],
                            EdgeWeightProperty(0),boost_graph->g );
        }
        if (node_map[children(0)]->depth==0){
            boost_graph->leaf_connected = true;
            for (int j=0; j<children.size(); ++j)
                add_edge( boost_graph->p[0], boost_graph->p[children(j)],
                            EdgeWeightProperty(0),boost_graph->g );
        }
            
        for (int i=0; i<parents.size(); ++i){
            for (int j=0; j<children.size(); ++j){
                // graph_metrics(0)++;
                if ( isEdge(node_map, parents(i), children(j)) ){
                    // graph_metrics(1)++;
                    // edges.push_back( Edge(parents(i),children(j)) );
                    double cost = computeGCost(node_map, parents(i), children(j));
                    // weights.push_back( cost );
                    add_edge( boost_graph->p[parents(i)],
                                boost_graph->p[children(j)],EdgeWeightProperty(cost),boost_graph->g );
                    atleast_one_edge = true;
                }
            }
        }
        return atleast_one_edge;
    };

void InsertNode(ikHandler* ik_handler, boost_graph* boost_graph, std::vector<node*>& node_map,
                        std::vector<Eigen::VectorXi>& node_list, int node_id){
    if (node_map[node_id]->depth < boost_graph->no_levels-1){
        Eigen::VectorXi parents(1);
        parents << node_id;
        MakeConnections( parents,node_list[node_map[node_id]->depth+1],ik_handler,node_map, boost_graph );
    }

    if (node_map[node_id]->depth > 0){
        Eigen::VectorXi children(1);
        children << node_id;
        MakeConnections( node_list[node_map[node_id]->depth-1],children,ik_handler,node_map, boost_graph );
    }
}

node* generate_node(Eigen::VectorXd joint_cfg,
                    int node_id, int node_depth, Eigen::VectorXd waypoint,
                    ikHandler* ik_handler, int row_id){
    
    node* new_node = new node;
    new_node->id = node_id;
    new_node->jt_config = joint_cfg;
    new_node->depth = node_depth;
    new_node->wp = waypoint;
    new_node->row_id = row_id;
    // KDL::Jacobian jac_kdl;
    // KDL::JntArray theta = DFMapping::Eigen_to_KDLJoints(joint_cfg);
    // ik_handler->robot->Jac_KDL(theta,jac_kdl);
    // auto jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    // new_node->jacobian = jac;
    new_node->wp_qt = GetQTWp(waypoint);
    return new_node;
};


int PickSource( std::vector<std::vector<int>>& unvisited_src,
                    std::vector<Eigen::MatrixXd>& ff_frames, int depth, bool src_bias ){
    int ff_frame_id = -1;
    int src_id;
    if (depth==0)
        src_id = 0;
    else
        src_id = 1;

    // // Pick the first source
    if (!src_bias){
        // ROS_WARN_STREAM("EXPLORATION");
        // Generate a random ff_frame index
        int index = 0 + ( std::rand() % ( unvisited_src[src_id].size() ) );
        ff_frame_id = unvisited_src[src_id][index];
        vector<int>::iterator it = unvisited_src[src_id].begin() + index;
        unvisited_src[src_id].erase( it );
    }
    else{
        // ROS_WARN_STREAM("EXPLOITATION");
        // Nabo::NNSearchD* tree;
        double max_cost;
        double min_cost;
        int no_wps;
        Eigen::MatrixXd wps;
        if (src_id==0){
            no_wps = src_waypoints.cols();
            // tree = Nabo::NNSearchD::createKDTreeLinearHeap(src_waypoints);
            max_cost = src_cost[1];
            min_cost = src_cost[0];
            wps = src_waypoints;
        }
        if (src_id==1){
            no_wps = snk_waypoints.cols();
            // tree = Nabo::NNSearchD::createKDTreeLinearHeap(snk_waypoints);
            max_cost = snk_cost[1];
            min_cost = snk_cost[0];
            wps = snk_waypoints;
        }

        // Normalize costs across source points
        for (int i=0; i<src_costs[src_id].size(); ++i){
            if (src_costs[src_id](i) > 1e7)
                src_balls[src_id](i) = 1.0;
            else
                src_balls[src_id](i) = (src_costs[src_id](i)-min_cost)/(max_cost-min_cost);
        }

        double threshold = src_balls[src_id].mean()*neigh_thrld;
        double bad_threshold = src_balls[src_id].mean() + (1-src_balls[src_id].mean())*(1-neigh_thrld);
        if (reject_samples){
            for (int i=0; i<src_costs[src_id].size(); ++i){
                if (src_balls[src_id](i) > bad_threshold){
                    // Absolutely bad neighborhood
                    Eigen::VectorXi indices(n_neighs); // Frame IDs
                    Eigen::VectorXd dists2(n_neighs);
                    kdtrees[depth]->knn( wps.col(i),indices, dists2, n_neighs );
                    for (int j=0; j<n_neighs; ++j){
                        auto it = std::find( unvisited_src[src_id].begin(),
                            unvisited_src[src_id].end(), indices(j) );
                        if (it!=unvisited_src[src_id].end()){// Element found
                            int pos = it - unvisited_src[src_id].begin();
                            junk_yard[src_id].push_back( unvisited_src[src_id][pos] );
                            vector<int>::iterator itr = unvisited_src[src_id].begin() + pos;
                            unvisited_src[src_id].erase( itr );
                        }
                    }
                }
            }
        }
        std::vector<int> sampleable_ids;
        // std::cout<< "Selecting Cost Value: ";
        for (int i=0; i<src_balls[src_id].size(); ++i){
            if (src_balls[src_id](i) > threshold)
                continue;
            // std::cout<< src_costs[src_id](i) << ",  ";
            Eigen::VectorXi indices(n_neighs);
            Eigen::VectorXd dists2(n_neighs);
            kdtrees[depth]->knn( wps.col(i),indices, dists2, n_neighs );
            for (int j=0; j<n_neighs; ++j){
                auto it = std::find( unvisited_src[src_id].begin(),
                        unvisited_src[src_id].end(), indices(j) );
                if (it!=unvisited_src[src_id].end()){
                    int pos = it - unvisited_src[src_id].begin();
                    it = std::find( sampleable_ids.begin(),sampleable_ids.end(), pos );
                    if (it==sampleable_ids.end())
                        sampleable_ids.push_back( pos );
                }
            }
        }
        // std::cout<< std::endl;

        // for (auto id : sampleable_ids)
        //     std::cout<< id << ",  ";
        // std::cout<< "\n\n";

        // int itr = 0;
        // for (int i=0; i<unvisited_src[src_id].size(); ++i){
        //     // Calculate distance
        //     Eigen::VectorXi indices(1);
        //     Eigen::VectorXd dists2(1);
        //     tree->knn(GetQTWp(ff_frames[depth].row(unvisited_src[src_id][i]).transpose()), 
        //                 indices, dists2, 1);
        //     if (src_balls[src_id](indices(0)) > threshold)
        //         sampleable_ids.push_back( i );
        //     itr++;
        // }
        // delete tree;

        if (sampleable_ids.size()!=0){
            // ROS_WARN_STREAM("SUCCESSFULLY EXPLOITED" << ". Sampleable size: " << sampleable_ids.size()
            //     << ". threshold value: " << threshold);
            // Generate a random ff_frame index
            int index = 0 + ( std::rand() % ( sampleable_ids.size() ) );
            ff_frame_id = unvisited_src[src_id][sampleable_ids[index]];
            vector<int>::iterator it = unvisited_src[src_id].begin() + sampleable_ids[index];
            unvisited_src[src_id].erase( it );
        }
        else{
            if (unvisited_src[src_id].size()!=0){
                // Generate a random ff_frame index
                int index = 0 + ( std::rand() % ( unvisited_src[src_id].size() ) );
                ff_frame_id = unvisited_src[src_id][index];
                vector<int>::iterator it = unvisited_src[src_id].begin() + index;
                unvisited_src[src_id].erase( it );
            }
        }
    }
    return ff_frame_id;
}



// bool GreedySamples(std::vector<std::vector<int>>& unvisited_src,
//                     std::vector<Eigen::MatrixXi>& isCreated,
//                     std::vector<Eigen::MatrixXd>& ff_frames, int depth, Eigen::VectorXd& waypoint,
//                     int& optID, Eigen::VectorXd& graph_metrics, bool isSource, int& trial_itr, bool src_bias,
//                     bool near_src ){
//     if (isSource){
//         int ff_frame_id;
//         int src_id;
        // if (depth==0)
        //     src_id = 0;
        // else
        //     src_id = 1;

//         // Pick neighbors of source
//         if (near_src==true){
//             // OptID defined here
//             Eigen::VectorXi indices(trial_itr+1);
//             Eigen::VectorXd dists2(trial_itr+1);
//             kdtrees[depth]->knn(prev_wp, indices, dists2, trial_itr+1);
//             ff_frame_id = indices(trial_itr);
//             trial_itr++;
//             unvisited_src[src_id].erase( std::find(unvisited_src[src_id].begin(), 
//                                     unvisited_src[src_id].end(), ff_frame_id) );
//         }
//         else{
//             // Pick the first source
//             if (!src_bias){
//                 // Generate a random ff_frame index
//                 int index = 0 + ( std::rand() % ( unvisited_src[src_id].size() ) );
//                 ff_frame_id = unvisited_src[src_id][index];
//                 vector<int>::iterator it = unvisited_src[src_id].begin() + index;
//                 unvisited_src[src_id].erase( it );
//             }
//             else{
//                 Eigen::VectorXd ref;
//                 if (src_id==0)
//                     ref = best_source;
//                 if (src_id==1)
//                     ref = best_sink;

//                 int itr = 0;
//                 int best_id;
//                 double max_dist = 0;
//                 while(itr < 1000){
//                     // Generate a random ff_frame index
//                     int index = 0 + ( std::rand() % ( unvisited_src[src_id].size() ) );
//                     ff_frame_id = unvisited_src[src_id][index];
//                     // Calculate distance
//                     double dist = (GetQTWp(ff_frames[depth].row(ff_frame_id).transpose()) - GetQTWp(ref)).norm();
//                     if (dist < 0.4){
//                         if (dist > max_dist){
//                             max_dist = dist;
//                             best_id = ff_frame_id;
//                         }
//                     }
//                     else{
//                         best_id = ff_frame_id;
//                         break;
//                     }
//                     itr++;
//                 }
//                 ff_frame_id = best_id;
//                 unvisited_src[src_id].erase( std::find(unvisited_src[src_id].begin(), 
//                                     unvisited_src[src_id].end(), ff_frame_id) );
//             }
//         }

//         optID = ff_frame_id;
        // if (isCreated[depth](optID,0) == 0) // Under collision
        //     return false;

//         // Get the corresponding waypoint
//         waypoint = ff_frames[depth].row(ff_frame_id).transpose();
        
//         return true;
//     }
    
//     // To speed up add an option to remove element from kdtree
    // // OptID defined here
    // Eigen::VectorXi indices(trial_itr+1);
    // Eigen::VectorXd dists2(trial_itr+1);
    // kdtrees[depth]->knn(prev_wp, indices, dists2, trial_itr+1);
    // optID = indices(trial_itr);
//     trial_itr++;
//     if (isCreated[depth](optID,0) == 0)
//         return false;
    
//     waypoint = ff_frames[depth].row(optID).transpose();
    
//     return true;
// };

public:
Eigen::VectorXd best_source;
Eigen::VectorXd best_sink;
std::vector<Nabo::NNSearchD*> kdtrees;
std::vector<Eigen::MatrixXd> M;
Eigen::MatrixXi graph_stats;
bool infeasibility;
int max_samples;
int attempts;
int src_cnt;
Eigen::MatrixXd src_waypoints;
std::vector<Eigen::VectorXd> src_balls;
std::vector<Eigen::VectorXd> src_costs;
Eigen::MatrixXd snk_waypoints;
Eigen::VectorXd prev_wp;
double radius;
std::vector<std::vector<int>> junk_yard;
std::vector<double> d_list;
std::vector<double> src_cost;
std::vector<double> snk_cost;
double neigh_thrld;
int n_neighs;
std::vector<Eigen::VectorXd> src_ids;
bool reject_samples;

FFSampler(){
    infeasibility = false; attempts = 0; src_cnt = 0; src_balls.resize(2); radius = 0.01;
    src_costs.resize(2); src_cost.resize(2); snk_cost.resize(2); 
    src_cost[0] = std::numeric_limits<float>::infinity(); src_cost[1] = 0;
    snk_cost[0] = std::numeric_limits<float>::infinity(); snk_cost[1] = 0;
    src_ids.resize(2);
    junk_yard.resize(2);
    reject_samples = true;
    if(!ros::param::get("/neigh_thrld",neigh_thrld)){
        std::cout<< "Unable to Good nodes threshold\n";
        neigh_thrld = 0.8;
    }

    if(!ros::param::get("/k",n_neighs)){
        std::cout<< "Unable to Obtain Maximum Iterations. Setting Default\n";
        n_neighs = 0.1;
    }
    }
~FFSampler(){};

Eigen::VectorXd GetQTWp(Eigen::VectorXd waypoint){
    Eigen::VectorXd wp_qt(7);
    wp_qt.segment(0,3) = waypoint.segment(0,3);
    wp_qt.segment(3,4) = rtf::bxbybz2qt(waypoint.segment(3,9).transpose()).row(0).transpose();
    return wp_qt;
};

Eigen::VectorXd GetWp(Eigen::VectorXd waypoint){
    Eigen::VectorXd wp(12);
    wp.segment(0,3) = waypoint.segment(0,3);
    wp.segment(3,9) = rtf::eul2bxbybz(rtf::qt2eul(waypoint.segment(3,4).transpose(), "ZYX")).row(0).transpose();
    return wp;  
}


bool GenNodeSamples(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter,
                    std::vector<std::vector<int>>& unvisited_src, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list,  int depth, Eigen::MatrixXi& sampled_wps,
                    std::vector<Eigen::MatrixXi>& isCreated, Eigen::VectorXd& graph_metrics, 
                    bool isSource, boost_graph* boost_graph, bool src_bias, Eigen::VectorXd& src_wp, int& id){
    Eigen::VectorXi optID;
    Eigen::VectorXd waypoint;
    if (isSource){
        optID.conservativeResize(1);
        optID(0) = PickSource( unvisited_src, ff_frames, depth, src_bias);
        if (optID(0)==-1)
            return false;
        // optID(0) = 680; // Gear source
        // optID(0) = 717; // Bath tub source
        // optID(0) = 233; // Gear source
        // optID(0) = 144; // Bath tub source
        // optID(0) = 19; // Boeing
        src_wp = GetQTWp(ff_frames[depth].row(optID(0)).transpose());
        id = optID(0);        
        src_cnt++;
    }
    else{ // PickNode
        int trial_itr = 0;
        double closest_distance = -1;
        while (trial_itr<ff_frames[depth].rows()){
            // Find nearest neighbor to previous waypoint
            Eigen::VectorXi indices(trial_itr+1);
            Eigen::VectorXd dists2(trial_itr+1);
            kdtrees[depth]->knn(prev_wp, indices, dists2, trial_itr+1);
            // kdtrees[depth]->knn(prev_wp.segment(0,3), indices, dists2, trial_itr+1);

            if (isCreated[depth](indices(trial_itr),0) == 0){ // Under collision
                trial_itr++;
                continue;
            }

            if (closest_distance < 0)
                closest_distance = dists2(trial_itr);

            if (std::fabs(dists2(trial_itr)-closest_distance) < 1e-8){
                optID.conservativeResize(optID.size()+1);
                optID(optID.size()-1) = indices(trial_itr);
                // std::cout<< dists2(trial_itr) << ",  ";
                trial_itr++;
                continue;
            }

            // if (optID.size() > resource)
            break;
        }
        // std::cout<< "\n";
    }
    bool samples_found = false;
    for (int i=0; i<optID.size(); ++i){
        if (isCreated[depth](optID(i),0) == 0) // Under collision
            continue;

        if (isCreated[depth](optID(i),0)==1){
            samples_found = true;
            sampled_wps.conservativeResize(sampled_wps.rows()+1,isCreated[depth].cols());
            sampled_wps.row(sampled_wps.rows()-1) << optID(i),isCreated[depth].block(optID(i),1,1,8);
            continue;
        }

        // Solve IK and check for collision
        // If valid solution, then create a node out of it
        int no_sols = 0;
        isCreated[depth](optID(i),0) = 0; // If this is not set back to 1 then wp is bogus
        waypoint = ff_frames[depth].row(optID(i)).transpose();
        if ( ik_handler->solveIK(waypoint) ){
            // For every solution create a node
            for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
                attempts++;
                // graph_metrics(2)++;
                // Check for collision
                std::vector<Eigen::MatrixXd> fk_kdl = 
                ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                if (geo_filter->is_tool_collision_free_(waypoint)){
                // if (true){
                    if(!wm->inCollision( fk_kdl )){
                    // if (true){
                        // graph_metrics(3)++;
                        int node_id = node_map.size();
                        node* new_node = generate_node(ik_handler->solution.col(sol_no), 
                                    node_id, depth, waypoint,ik_handler, optID(i));
                        node_map.push_back(new_node);
                        boost_graph->p.push_back(vertex(node_id,boost_graph->g));
                        InsertNode(ik_handler,boost_graph,node_map,node_list,node_id);
                        node_list[depth].conservativeResize(node_list[depth].size()+1);
                        node_list[depth](node_list[depth].size()-1) = node_id;
                        isCreated[depth](optID(i),sol_no+1) = node_id;
                        isCreated[depth](optID(i),0) = 1; // Ik exists here
                    }
                }
            }
        }

        if (isCreated[depth](optID(i),0)==1){
            samples_found = true;
            sampled_wps.conservativeResize(sampled_wps.rows()+1,isCreated[depth].cols());
            sampled_wps.row(sampled_wps.rows()-1) << optID(i),isCreated[depth].block(optID(i),1,1,8);
            continue;
        }
    }
    return samples_found;
}



// Eigen::VectorXi GenNodeSamples(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
//                     WM::WM* wm, GeometricFilterHarness* geo_filter,
//                     std::vector<std::vector<int>>& unvisited_src, std::vector<node*>& node_map,
//                     std::vector<Eigen::VectorXi>& node_list,  int depth, 
//                     std::vector<Eigen::MatrixXi>& isCreated, Eigen::VectorXd& graph_metrics, 
//                     bool isSource, boost_graph* boost_graph, bool src_bias, int resource){
//     int optID;
//     Eigen::VectorXi sampled_nodes;
//     Eigen::VectorXd waypoint;
//     int trial_itr = 0;
//     int itr = 0;
//     bool near_src = false;
//     while (trial_itr < ff_frames[depth].rows()-1 && itr < resource){
//         if (itr > 0)
//             near_src = true;

//         if ( !GreedySamples(unvisited_src,isCreated, ff_frames,depth,waypoint,
//                             optID, graph_metrics, isSource, trial_itr, src_bias, near_src) ){
//             itr++;
//             continue;
//         }

//         // Check if a node has already been created
        // if (isCreated[depth](optID,0) == 1){
        //     if (itr==0){
        //         prev_wp = GetQTWp(waypoint);
        //         prev_index = optID;
        //         prev_depth = depth;
        //     }
        //     // Node already exists so we return the existing node ids
        //     for (int i=1; i<isCreated[depth].cols(); ++i){
        //         if ( isCreated[depth](optID,i)!=-1 ){
        //             sampled_nodes.conservativeResize(sampled_nodes.size()+1);
        //             sampled_nodes(sampled_nodes.size()-1) = isCreated[depth](optID,i);
        //         }
        //     }
        //     itr++;
        //     continue;
        // }

//         if (itr==0){
//             prev_wp = GetQTWp(waypoint);
//             prev_index = optID;
//             prev_depth = depth;
//         }
//         // Solve IK and check for collision
//         // If valid solution, then create a node out of it
//         int no_sols = 0;
//         isCreated[depth](optID,0) = 0; // If this is not set back to 1 then wp is bogus
//         if ( ik_handler->solveIK(waypoint) ){
//             // For every solution create a node
//             for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
//                 // graph_metrics(2)++;
//                 // Check for collision
//                 std::vector<Eigen::MatrixXd> fk_kdl = 
//                 ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
//                 if (geo_filter->is_tool_collision_free_(waypoint)){
//                 // if (true){
//                     if(!wm->inCollision( fk_kdl )){
//                     // if (true){
//                         // graph_metrics(3)++;
//                         int node_id = node_map.size();
//                         node* new_node = generate_node(ik_handler->solution.col(sol_no), 
//                                             node_id, depth, waypoint,ik_handler, optID);
//                         node_map.push_back(new_node);
                        // boost_graph->p.push_back(vertex(node_id,boost_graph->g));
                        // InsertNode(ik_handler,boost_graph,node_map,node_list,node_id);
//                         sampled_nodes.conservativeResize(sampled_nodes.size()+1);
//                         sampled_nodes(sampled_nodes.size()-1) = node_id;
//                         node_list[depth].conservativeResize(node_list[depth].size()+1);
//                         node_list[depth](node_list[depth].size()-1) = node_id;
//                         isCreated[depth](optID,sol_no+1) = node_id;
//                         isCreated[depth](optID,0) = 1; // Ik exists here
//                     }
//                 }
//             }
//         }
//         itr++;
//     }
//     return sampled_nodes;
// };


bool RandomSample(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list,  
                    int depth, std::vector<Eigen::MatrixXi>& isCreated,
                    Eigen::VectorXd& graph_metrics, boost_graph* boost_graph, double resource){
    Eigen::VectorXi sampled_nodes;
    Eigen::VectorXd waypoint;
    std::vector<int> unvisited_samples; unvisited_samples.clear();
    for (int i=0; i<ff_frames[depth].rows();++i){
        if (isCreated[depth](i,0)==-1)
            unvisited_samples.push_back(i);
    }

    int itr = 0;
    int max_resc = resource;
    while (unvisited_samples.size()!=0 && itr < max_resc){
        // Generate a random ff_frame index
        int index = 0 + ( std::rand() % ( unvisited_samples.size() ) );
        auto ff_frame_id = unvisited_samples[index];
        vector<int>::iterator it = unvisited_samples.begin() + index;
        unvisited_samples.erase( it );
        
        waypoint = ff_frames[depth].row(ff_frame_id).transpose();

        // Solve IK and check for collision
        // If valid solution, then create a node out of it
        int no_sols = 0;
        isCreated[depth](ff_frame_id,0) = 0; // If this is not set back to 1 then wp is bogus
        if ( ik_handler->solveIK(waypoint) ){
            // For every solution create a node
            for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
                attempts++;
                // graph_metrics(2)++;
                // Check for collision
                std::vector<Eigen::MatrixXd> fk_kdl = 
                ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                if (geo_filter->is_tool_collision_free_(waypoint)){
                // if (true){
                    if(!wm->inCollision( fk_kdl )){
                    // if (true){
                        // graph_metrics(3)++;
                        int node_id = node_map.size();
                        node* new_node = generate_node(ik_handler->solution.col(sol_no), 
                                            node_id, depth, waypoint,ik_handler, ff_frame_id);
                        node_map.push_back(new_node);
                        boost_graph->p.push_back(vertex(node_id,boost_graph->g));
                        InsertNode(ik_handler,boost_graph,node_map,node_list,node_id);
                        sampled_nodes.conservativeResize(sampled_nodes.size()+1);
                        sampled_nodes(sampled_nodes.size()-1) = node_id;
                        node_list[depth].conservativeResize(node_list[depth].size()+1);
                        node_list[depth](node_list[depth].size()-1) = node_id;
                        isCreated[depth](ff_frame_id,sol_no+1) = node_id;
                        isCreated[depth](ff_frame_id,0) = 1;
                    }
                }
            }
        }
        itr++;
    }
    // if (sampled_nodes.size()==0){
    //     ROS_WARN_STREAM("**********ALERT***************");
    //     ROS_WARN_STREAM("All nodes evaluated");
    // }

    if (sampled_nodes.size()!=0)
        return true;
    else
        return false;
};

void NearestNode(ikHandler* ik_handler, WM::WM* wm, Eigen::VectorXd waypoint,
                std::vector<Eigen::MatrixXd>& ff_frames, int depth,
                std::vector<Eigen::MatrixXi>& isCreated,
                boost_graph* boost_graph,
                GeometricFilterHarness* geo_filter, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list, double resource){
    // int trial_itr = 0;
    // int optID;
    // bool node_created = false;
    // int counter = 0;
    // Eigen::MatrixXd wps(101,12);
    // wps.row(0) = waypoint.transpose();
    // int no_neigh = ceil(resource*ff_frames[depth].rows());
    int no_neigh = resource;
    Eigen::VectorXd wp_quat = GetQTWp(waypoint);
    // while( (trial_itr < ff_frames[depth].rows()-1) && !node_created ){
    // while( counter < 2000 && trial_itr < ff_frames[depth].rows()-1 ){
    Eigen::VectorXi indices(no_neigh);
    Eigen::VectorXd dists2(no_neigh);
    kdtrees[depth]->knn(wp_quat, indices, dists2, no_neigh);

    // for (int i=0; i<indices.size(); ++i){
    //     wps.row(i+1) = GetWp(ff_frames[depth].row(indices(i)).transpose()).transpose();
    // }

    for (int i=0; i<indices.size(); ++i){
        int optID = indices(i);
        if (isCreated[depth](optID,0) == 0)
            continue;
        
        if (isCreated[depth](optID,0) == 1)
            continue;

        if (isCreated[depth](optID,0) == -1){
            // Create the node
            // Solve IK and check for collision
            // If valid solution, then create a node out of it
            Eigen::VectorXd neigh_wp = ff_frames[depth].row(optID).transpose();
            int no_sols = 0;
            isCreated[depth](optID,0) = 0; // If this is not set back to 1 then wp is under collision
            if ( ik_handler->solveIK(neigh_wp) ){
                // For every solution create a node
                for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
                    attempts++;
                    // Check for collision
                    std::vector<Eigen::MatrixXd> fk_kdl = 
                    ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                    if (geo_filter->is_tool_collision_free_(neigh_wp)){
                        if(!wm->inCollision( fk_kdl )){
                            int node_id = node_map.size();
                            node* new_node = generate_node(ik_handler->solution.col(sol_no), 
                                                node_id, depth, neigh_wp, ik_handler, optID);
                            node_map.push_back(new_node);
                            boost_graph->p.push_back(vertex(node_id,boost_graph->g));
                            InsertNode(ik_handler,boost_graph,node_map,node_list,node_id);
                            node_list[depth].conservativeResize(node_list[depth].size()+1);
                            node_list[depth](node_list[depth].size()-1) = node_id;
                            isCreated[depth](optID,sol_no+1) = node_id;
                            isCreated[depth](optID,0) = 1;
                        }
                    }
                }
            }
        }
    }
    // std::string csv_dir;
    // csv_dir = ros::package::getPath("pct") + "/data/csv/";
    // file_rw::file_write(csv_dir+"../test_case_specific_data/frames" + std::to_string(depth) + ".csv",
    //                             ff_frames[depth]);
};

};
#endif