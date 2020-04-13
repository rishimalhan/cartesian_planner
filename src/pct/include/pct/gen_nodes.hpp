///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __gen_nodes_hpp__
#define __gen_nodes_hpp__

#include <robot_utilities/ikHandler.hpp>
#include <pct/node_description.hpp>
#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>

bool gen_nodes(ikHandler* ik_handler, WM::WM* wm,
                const std::vector<Eigen::MatrixXd>& wpTol, const std::vector<Eigen::MatrixXd>& tcp_list,
                std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list){

    std::cout<< "\n\n##############################################################\n";
    std::cout<< "Generating nodes\n";
    node_list.clear();
    // node_map will carry descriptions of all nodes
    // node_list will carry the structure. i.e node ids at every depth which will be used to build graph
    int id_cnt = 0;
    for (int i=0; i<wpTol.size();++i){ // For all the waypoints
        Eigen::MatrixXd waypoints = wpTol[i];
        Eigen::VectorXi nodes_at_depth;
        int ctr = 0;
        for (int j=0; j<waypoints.rows(); ++j){
            // For each tcp solve IK and create node
            for (int k=0; k<tcp_list.size();++k){
                ik_handler->setTcpFrame(tcp_list[k]);
                // Solve IK and check for collision
                // If valid solution, then create a node out of it
                if (ik_handler->solveIK(waypoints.row(j).transpose())){
                    // For every solution create a node
                    for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
                        // Check for collision
                        std::vector<Eigen::MatrixXd> fk_kdl = ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                        if(!wm->inCollision( fk_kdl )){
                            node* curr_node = new node;
                            curr_node->id = id_cnt;
                            curr_node->jt_config = ik_handler->solution.col(sol_no);
                            curr_node->depth = i;
                            curr_node->index = j;
                            curr_node->tcp_id = k;
                            node_map.push_back(curr_node);
                            // Add these node ids to the node_list
                            nodes_at_depth.conservativeResize(ctr+1);
                            nodes_at_depth(ctr) = curr_node->id; ctr++;
                            id_cnt++;
                        }
                    }
                }
            }
        }
        if (nodes_at_depth.size()==0){
            std::cout<< "All waypoints unreachable within tolerances at index: " << i << ". Base index is 0. Terminating\n";
            std::cout<< "##############################################################\n\n";
            return false;
        }
        else{
            // std::cout<< "Node ids: " << nodes_at_depth.transpose() << "\n\n";
            node_list.push_back(nodes_at_depth);
        }
    }

    // for (int i=0; i<node_list.size(); ++i)
    //     std::cout<< node_list[i].transpose() << "\n";

    std::cout<< "Total Number of Nodes: " << id_cnt << "\n"; 
    std::cout<< "##############################################################\n\n";
    return true;
};


#endif