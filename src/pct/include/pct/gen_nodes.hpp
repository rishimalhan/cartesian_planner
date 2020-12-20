///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __gen_nodes_hpp__
#define __gen_nodes_hpp__

#include <pct/node_description.hpp>
#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/transformation_utilities.hpp>
#include <pct/geometric_filter.h>

bool gen_nodes(ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
                std::vector<Eigen::MatrixXd>& ff_frames,
                std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                Eigen::MatrixXd& success_flags){
    std::cout<< "\n\n##############################################################\n";
    std::cout<< "Generating nodes\n";
    int NumWaypoints = ff_frames.size();
    node_list.clear();
    success_flags = Eigen::MatrixXd::Ones(NumWaypoints,1)*1; // All reachable. Mark them 0 if they are not
    bool status = true;
    
    // Dummy root and leaf
    node* root_node = new node;
    root_node->id = 0;
    root_node->depth = -1;
    node_map.push_back(root_node);

    node* leaf_node = new node;
    leaf_node->id = 1;
    leaf_node->depth = ff_frames.size();
    node_map.push_back(leaf_node);

    // node_map will carry descriptions of all nodes
    // node_list will carry the structure. i.e node ids at every depth which will be used to build graph
    int id_cnt = 2;
    int tot_count = 0;
    // KDL::JntArray theta;
    // KDL::Jacobian jac_kdl;
    // Eigen::MatrixXd jac;
    ik_handler->setTcpFrame(Eigen::MatrixXd::Identity(4,4));
    for (int i=0; i<ff_frames.size(); ++i){
        int tot_nodes_lvl = 0;
        int valid_nodes_lvl = 0;
        Eigen::VectorXi nodes_at_depth;
        int ctr = 0;
        Eigen::MatrixXd waypoints = ff_frames[i];
        for (int j=0; j<waypoints.rows(); ++j){
            // Solve IK and check for collision
            // If valid solution, then create a node out of it
            int no_sols = 0;
            if (ik_handler->solveIK(waypoints.row(j).transpose())){
                // For every solution create a node
                for (int sol_no=0; sol_no<ik_handler->solution.cols(); ++sol_no){
                    tot_count++;
                    tot_nodes_lvl++;
                    // Check for collision
                    std::vector<Eigen::MatrixXd> fk_kdl = ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                    if( !wm->inCollision( fk_kdl ) && 
                        geo_filter->is_tool_collision_free_(waypoints.row(j).transpose()) ){
                    // if(true){
                        // theta = DFMapping::Eigen_to_KDLJoints(ik_handler->solution.col(sol_no));
                        // ik_handler->robot->Jac_KDL(theta,jac_kdl);
                        // jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
                        no_sols ++;
                        node* curr_node = new node;
                        curr_node->row_id = j;
                        curr_node->id = id_cnt;
                        curr_node->jt_config = ik_handler->solution.col(sol_no);
                        curr_node->depth = i;
                        curr_node->wp = waypoints.row(j).transpose();
                        // curr_node->jacobian = jac;

                        // // bxbybz to euler waypoint
                        // curr_node->wp_eul.resize(6);
                        // curr_node->wp_eul.segment(0,3) = waypoints.block(j,0,1,3).transpose();
                        // curr_node->wp_eul.segment(3,3) = rtf::bxbybz2eul(waypoints.block(j,3,1,9),"XYZ").row(0).transpose();
                        // curr_node->wp_quat.resize(7);
                        // curr_node->wp_quat.segment(0,3) = waypoints.block(j,0,1,3).transpose();
                        // curr_node->wp_quat.segment(3,4) = rtf::bxbybz2qt(waypoints.block(j,3,1,9)).row(0).transpose();
                        // // xyzbxbybz of flange wrt rob_base
                        // curr_node->wp_ff = ik_handler->getFFTarget();


                        node_map.push_back(curr_node);
                        // Add these node ids to the node_list
                        nodes_at_depth.conservativeResize(ctr+1);
                        nodes_at_depth(ctr) = curr_node->id; ctr++;
                        id_cnt++;
                        valid_nodes_lvl++;
                    }
                }
            }
        }
        if (nodes_at_depth.size()==0){
            std::cout<< "All waypoints unreachable within tolerances at index: " << i << ". Base index is 0.\n";
            success_flags(i) = 0;
            status = false;
            std::cout<< "##############################################################\n\n";
            return false;
        }
        else{
            // std::cout<< "Node ids: " << nodes_at_depth.transpose() << "\n\n";
            node_list.push_back(nodes_at_depth);
        }
        // std::cout << (double)valid_nodes_lvl / tot_nodes_lvl << "\n";
    }
    std::cout<< "Total Number of Valid Nodes: " << id_cnt << "\n"; 
    std::cout<< "Total Number of Nodes: " << tot_count << "\n";
    std::cout<< "##############################################################\n\n";
    return status;
};
#endif