#ifndef __SAMPLENODES__HPP__
#define __SAMPLENODES__HPP__

#include <pct/node_description.hpp>
#include <random>
#include <Eigen/Eigen>
#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/transformation_utilities.hpp>
#include <pct/geometric_filter.h>


class FFSampler{
private:
node* prev_node;

Eigen::VectorXd GetQTWp(Eigen::VectorXd waypoint){
    Eigen::VectorXd wp_qt(7);
    wp_qt.segment(0,3) = waypoint.segment(0,3);
    wp_qt.segment(3,4) = rtf::bxbybz2qt(waypoint.segment(3,9).transpose()).row(0).transpose();
    return wp_qt;
};

node* generate_node(Eigen::VectorXd joint_cfg, int family_id,
                    int node_id, int node_depth, Eigen::VectorXd waypoint,
                    ikHandler* ik_handler){
    
    node* new_node = new node;
    new_node->id = node_id;
    new_node->jt_config = joint_cfg;
    new_node->family_id = family_id;
    new_node->depth = node_depth;
    new_node->wp = waypoint;
    // KDL::Jacobian jac_kdl;
    // KDL::JntArray theta = DFMapping::Eigen_to_KDLJoints(joint_cfg);
    // ik_handler->robot->Jac_KDL(theta,jac_kdl);
    // auto jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    // new_node->jacobian = jac;
    new_node->wp_qt = GetQTWp(waypoint);
    return new_node;
};


bool sampling_scheme(std::vector<std::vector<int>>& unvisited_ids,
                                std::vector<Eigen::MatrixXd>& ff_frames, int depth,
                                Eigen::VectorXd& waypoint ){
    if (depth==0){
        // Generate a random ff_frame index
        int index = 0 + ( std::rand() % ( unvisited_ids[depth].size() ) );
        auto ff_frame_id = unvisited_ids[depth][index];
        vector<int>::iterator it = unvisited_ids[depth].begin() + index;
        unvisited_ids[depth].erase( it );
        // Get the corresponding waypoint
        waypoint = ff_frames[depth].row(ff_frame_id).transpose();
        return true;
    }


    // Generate Sample Based on Jacobian Control
    double max_dist = std::numeric_limits<double>::infinity();
    int opt_id;
    int ctr = 0;
    int del_ctr;
    for (int id : unvisited_ids[depth]){
        auto wp_qt = GetQTWp(ff_frames[depth].row(id).transpose());
        // auto prev_jac = prev_node->jacobian;
        auto prev_wp = prev_node->wp_qt;
        // auto dist = (prev_jac.inverse() * (wp_qt-prev_wp)).norm();
        auto dist = (wp_qt-prev_wp).norm();
        if ( dist < max_dist ){
            max_dist = dist;
            opt_id = id;
            del_ctr = ctr;
        }
        ctr++;
    }
    waypoint = ff_frames[depth].row(opt_id).transpose();
    vector<int>::iterator it = unvisited_ids[depth].begin() + del_ctr;
    unvisited_ids[depth].erase( it );
    return true;
};

public:
Eigen::MatrixXi graph_stats;
int max_samples;
FFSampler(int grph_depth){
    graph_stats = Eigen::MatrixXi::Zero(grph_depth,2);
};
~FFSampler(){};
bool GenNodeSamples(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter, std::vector<int>& sampled_nodes,
                    std::vector<std::vector<int>>& unvisited_ids, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list,  int depth){
    if (unvisited_ids[depth].size()==0)
        return false;
    if (depth>=node_list.size())
        node_list.resize(depth+1);
    sampled_nodes.clear();
    
    bool is_sample_gen = false;
    while (!is_sample_gen && unvisited_ids[depth].size()>0){
        Eigen::VectorXd waypoint;
        sampling_scheme(unvisited_ids,ff_frames,depth,waypoint);
        // Solve IK and check for collision
        // If valid solution, then create a node out of it
        int no_sols = 0;
        if ( ik_handler->solveIK(waypoint) ){
            // For every solution create a node
            for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
                /////////////////////////
                // Only one family for now
                if (ik_handler->ikFamily(sol_no)!=2)
                    continue;
                /////////////////////////

                // Check for collision
                std::vector<Eigen::MatrixXd> fk_kdl = 
                ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                if (geo_filter->is_tool_collision_free_(waypoint)){
                    if(!wm->inCollision( fk_kdl )){
                        int node_id = node_map.size();
                        node* new_node = generate_node(ik_handler->solution.col(sol_no), 
                                            ik_handler->ikFamily(sol_no),
                                            node_id, depth, waypoint,ik_handler);
                        prev_node = new_node;
                        node_map.push_back(new_node);
                        sampled_nodes.push_back( node_id );
                        node_list[depth].conservativeResize(node_list[depth].size()+1);
                        node_list[depth](node_list[depth].size()-1) = node_id;
                        is_sample_gen = true;
                    }
                    else{
                        graph_stats(depth,1)++;
                        // ROS_WARN_STREAM("Collision");
                    }
                }
                else{
                    // ROS_WARN_STREAM("Collision");
                    graph_stats(depth,1)++;
                }
            }
        }
        else{
            // ROS_WARN_STREAM("Limits Violated");
            graph_stats(depth,0)++;
        }
    }
    if (!is_sample_gen)
        return false;
    return true;
};
};
#endif