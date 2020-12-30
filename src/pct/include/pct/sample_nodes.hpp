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
                    std::vector<Eigen::MatrixXd>& ff_frames, int depth, bool diversity ){
    int ff_frame_id = -1;
    int src_id;
    if (depth==0)
        src_id = 0;
    else
        src_id = 1;

    // Pick the first source
    if (!diversity){
        // Generate a random ff_frame index
        int index = 0 + ( std::rand() % ( unvisited_src[src_id].size() ) );
        ff_frame_id = unvisited_src[src_id][index];
        vector<int>::iterator it = unvisited_src[src_id].begin() + index;
        unvisited_src[src_id].erase( it );
    }
    else{
        Nabo::NNSearchD* tree;
        if (src_id==0)
            tree = Nabo::NNSearchD::createKDTreeLinearHeap(src_waypoints);
        if (src_id==1)
            tree = Nabo::NNSearchD::createKDTreeLinearHeap(snk_waypoints);

        int itr = 0;
        int best_id;
        double max_dist = 0;
        while(itr < unvisited_src[src_id].size()){
            // Generate a random ff_frame index
            int index = 0 + ( std::rand() % ( unvisited_src[src_id].size() ) );
            ff_frame_id = unvisited_src[src_id][index];
            vector<int>::iterator it = unvisited_src[src_id].begin() + index;
            unvisited_src[src_id].erase( it );
            // Calculate distance
            Eigen::VectorXi indices(1);
            Eigen::VectorXd dists2(1);
            tree->knn(GetQTWp(ff_frames[depth].row(ff_frame_id).transpose()), indices, dists2, 1);
            double dist = dists2(0);
            if (dist > src_balls[src_id](indices(0))){
                best_id = ff_frame_id;
                break;
            }
            itr++;
        }
        ff_frame_id = best_id;
        delete tree;
    }

    if (ff_frame_id==-1)
        return -1;

    if (src_id==0){
        src_waypoints.conservativeResize(7,src_waypoints.cols()+1);
        src_waypoints.col(src_waypoints.cols()-1) = GetQTWp(ff_frames[depth].row(ff_frame_id).transpose());
    }

    if (src_id==1){
        snk_waypoints.conservativeResize(7,snk_waypoints.cols()+1);
        snk_waypoints.col(snk_waypoints.cols()-1) = GetQTWp(ff_frames[depth].row(ff_frame_id).transpose());
    }
    return ff_frame_id;
}



// bool GreedySamples(std::vector<std::vector<int>>& unvisited_src,
//                     std::vector<Eigen::MatrixXi>& isCreated,
//                     std::vector<Eigen::MatrixXd>& ff_frames, int depth, Eigen::VectorXd& waypoint,
//                     int& optID, Eigen::VectorXd& graph_metrics, bool isSource, int& trial_itr, bool diversity,
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
//             if (!diversity){
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
Eigen::MatrixXd snk_waypoints;
Eigen::VectorXd prev_wp;

FFSampler(){infeasibility = false; attempts = 0; src_cnt = 0; src_balls.resize(2);};
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


Eigen::VectorXi GenNodeSamples(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter,
                    std::vector<std::vector<int>>& unvisited_src, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list,  int depth, 
                    std::vector<Eigen::MatrixXi>& isCreated, Eigen::VectorXd& graph_metrics, 
                    bool isSource, boost_graph* boost_graph, bool diversity, int resource){
    Eigen::VectorXi optID;
    Eigen::VectorXi sampled_wps;
    Eigen::VectorXd waypoint;
    if (isSource){
        optID.conservativeResize(1);
        optID(0) = PickSource( unvisited_src, ff_frames, depth, diversity);
        // optID(0) = 233;
        src_cnt++;
        if (optID(0)==-1)
            return sampled_wps;
        if (isCreated[depth](optID(0),0) == 0) // Under collision
            return sampled_wps;
    }
    else{ // PickNode
        int trial_itr = 0;
        double closest_distance;
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
            if (trial_itr==0)
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

    if (optID.size()==0) // Set empty
        return sampled_wps;

    for (int i=0; i<optID.size(); ++i){
        if (isCreated[depth](optID(i),0) == 1)
            continue;
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
                        node_list[depth].conservativeResize(node_list[depth].size()+1);
                        node_list[depth](node_list[depth].size()-1) = node_id;
                        isCreated[depth](optID(i),sol_no+1) = node_id;
                        isCreated[depth](optID(i),0) = 1; // Ik exists here
                        boost_graph->p.push_back(vertex(node_id,boost_graph->g));
                    }
                }
            }
            if (isCreated[depth](optID(i),0)==1){
                sampled_wps.conservativeResize(sampled_wps.size()+1);
                sampled_wps(sampled_wps.size()-1) = optID(i);
            }
        }
    }
    return sampled_wps;
}



// Eigen::VectorXi GenNodeSamples(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
//                     WM::WM* wm, GeometricFilterHarness* geo_filter,
//                     std::vector<std::vector<int>>& unvisited_src, std::vector<node*>& node_map,
//                     std::vector<Eigen::VectorXi>& node_list,  int depth, 
//                     std::vector<Eigen::MatrixXi>& isCreated, Eigen::VectorXd& graph_metrics, 
//                     bool isSource, boost_graph* boost_graph, bool diversity, int resource){
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
//                             optID, graph_metrics, isSource, trial_itr, diversity, near_src) ){
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
//                         sampled_nodes.conservativeResize(sampled_nodes.size()+1);
//                         sampled_nodes(sampled_nodes.size()-1) = node_id;
//                         node_list[depth].conservativeResize(node_list[depth].size()+1);
//                         node_list[depth](node_list[depth].size()-1) = node_id;
//                         isCreated[depth](optID,sol_no+1) = node_id;
//                         isCreated[depth](optID,0) = 1; // Ik exists here
//                         boost_graph->p.push_back(vertex(node_id,boost_graph->g));
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
                    Eigen::VectorXd& graph_metrics, boost_graph* boost_graph, int resource){
    Eigen::VectorXi sampled_nodes;
    Eigen::VectorXd waypoint;
    std::vector<int> unvisited_samples; unvisited_samples.clear();
    for (int i=0; i<ff_frames[depth].rows();++i)
        unvisited_samples.push_back(i);
    int itr = 0;
    while (unvisited_samples.size()!=0 && itr < resource){
        // Generate a random ff_frame index
        int index = 0 + ( std::rand() % ( unvisited_samples.size() ) );
        auto ff_frame_id = unvisited_samples[index];
        vector<int>::iterator it = unvisited_samples.begin() + index;
        unvisited_samples.erase( it );
        if (isCreated[depth](ff_frame_id,0)!=-1) // Only proceed if node is unevaluated
            continue;
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
                        sampled_nodes.conservativeResize(sampled_nodes.size()+1);
                        sampled_nodes(sampled_nodes.size()-1) = node_id;
                        node_list[depth].conservativeResize(node_list[depth].size()+1);
                        node_list[depth](node_list[depth].size()-1) = node_id;
                        isCreated[depth](ff_frame_id,sol_no+1) = node_id;
                        isCreated[depth](ff_frame_id,0) = 1;
                        boost_graph->p.push_back(vertex(node_id,boost_graph->g));
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
                    std::vector<Eigen::VectorXi>& node_list, int no_neigh){
    // int trial_itr = 0;
    // int optID;
    // bool node_created = false;
    // int counter = 0;
    // Eigen::MatrixXd wps(101,12);
    // wps.row(0) = waypoint.transpose();
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
                            node_list[depth].conservativeResize(node_list[depth].size()+1);
                            node_list[depth](node_list[depth].size()-1) = node_id;
                            isCreated[depth](optID,sol_no+1) = node_id;
                            isCreated[depth](optID,0) = 1;
                            boost_graph->p.push_back(vertex(node_id,boost_graph->g));
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