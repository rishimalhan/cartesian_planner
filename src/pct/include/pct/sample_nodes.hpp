#ifndef __SAMPLENODES__HPP__
#define __SAMPLENODES__HPP__

#include <pct/node_description.hpp>
#include <random>
#include <Eigen/Eigen>
#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/transformation_utilities.hpp>


node* generate_node(Eigen::VectorXd joint_cfg, int family_id,
                    int node_id, int node_depth, Eigen::VectorXd waypoint){
    
    node* new_node = new node;
    new_node->id = node_id;
    new_node->jt_config = joint_cfg;
    new_node->family_id = family_id;
    new_node->depth = node_depth;
    new_node->wp = waypoint;
    return new_node;
};


bool sampling_scheme(std::vector<std::vector<int>>& unvisited_ids,
                                std::vector<Eigen::MatrixXd>& ff_frames, int depth,
                                Eigen::VectorXd& waypoint ){
    // Generate a random ff_frame index
    int index = 0 + ( std::rand() % ( unvisited_ids[depth].size() ) );
    auto ff_frame_id = unvisited_ids[depth][index];
    vector<int>::iterator it = unvisited_ids[depth].begin() + index;
    unvisited_ids[depth].erase( it );
    // Get the corresponding waypoint
    waypoint = ff_frames[depth].row(ff_frame_id).transpose();
    return true;
};

bool GenNodeSamples(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, std::vector<int>& sampled_nodes,
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
            // for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
            for (int sol_no=0; sol_no<1;++sol_no){ // Only one family for now
                // Check for collision
                std::vector<Eigen::MatrixXd> fk_kdl = ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                if(!wm->inCollision( fk_kdl )){
                    int node_id = node_map.size();
                    node* new_node = generate_node(ik_handler->solution.col(sol_no), 
                                        ik_handler->ikFamily(sol_no),
                                        node_id, depth, waypoint);
                    node_map.push_back(new_node);
                    sampled_nodes.push_back( node_id );
                    node_list[depth].conservativeResize(node_list[depth].size()+1);
                    node_list[depth](node_list[depth].size()-1) = node_id;
                    is_sample_gen = true;
                }
            }
        }
    }
    return true;
};

#endif