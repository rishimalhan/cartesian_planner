// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef __SS_SEARCHES__
#define __SS_SEARCHES__

#include <iostream>
#include <robot_utilities/ikHandler.hpp>
#include <Eigen/Eigen>
#include <queue>
#include <robot_utilities/priority_queue.hpp>
#include <ros/ros.h>
#include <robot_utilities/file_rw.hpp>
#include <limits>
#include <pct/gen_wdg.hpp>
#include <robot_utilities/world_manager.hpp>

class ss_searches{
private:
    // dji_v1 members
    struct djk_node{
        int id;
        int depth; // Waypoint ID
        int index; // Index in the tolerance matrix
        int parent_id;
        Eigen::VectorXd parent_config;
        double cost;
        Eigen::VectorXd jt_config;
        bool terminal = false;
        // Eigen::VectorXd waypoint;
    };
    std::vector<bool> visitedNodes;
    std::vector<djk_node*> node_map;
public:
    ss_searches(){
        node_map.clear();
    };
    ~ss_searches(){};


    // djk_v1 (djikstra) uses one matrix to store visited nodes. Nodes are basically a struct.
    // The two matrices are initialized in the constructor when tolerance path and ik_handler is passed
    bool djk_v1(ikHandler *ik_handler, WM::WM* wm, std::vector<Eigen::MatrixXd>& wpTol, Eigen::MatrixXd& ret_val){
        Eigen::MatrixXd trajectory(wpTol.size(),ik_handler->OptVarDim);

        std::cout<< "\n\n###########################################################\n";

        std::cout<< "Running Djikstra Search.......\n";
        bool reach_goal;
        // For each redundant configuration
        double lowest_cost = std::numeric_limits<double>::infinity();
        int goal_id;
        Eigen::MatrixXd rob_home = ik_handler->init_guess;

        std::cout<< "Number of Initial Configurations: " << wpTol[0].rows() << "\n\n";
        for (int i=0; i<wpTol[0].rows();++i){
        // for (int wp=0, i=wp; i<wp+1; ++i){
            // Check if IK exists for the waypoint. Initial guess can be zero or biased to rob home
            // ik_handler->init_guess = Eigen::VectorXd::Ones(ik_handler->OptVarDim)*0;
            ik_handler->init_guess = rob_home;
            if (!ik_handler->solveIK(wpTol[0].row(i))){
                std::cout<< "IK failed for configuration: " << i << "\n";
                continue;
            }
            else{
                std::cout<< "IK successful for configuration: " << i << " Attempting Trajectory Search\n";
                ik_handler->init_guess = ik_handler->closest_sol;
            }

            std::vector<std::vector<Eigen::VectorXi>> WDG; WDG.clear();
            std::vector<std::vector<int>> node_ids; node_ids.clear();
            std::vector<Eigen::MatrixXd> wpTol_filtered;  wpTol_filtered.clear();
            int node_cnt;
            if(!gen_WDG(ik_handler, wpTol, i, node_ids, node_cnt, WDG, wpTol_filtered)) // output is WDG
                continue;

            std::cout<< "\n\n#################################################\n\n";
            Eigen::MatrixXd curr_traj(wpTol_filtered.size(),ik_handler->OptVarDim);
            int max_depth = -1;
            reach_goal = false;
            // Conduct the Search For a Path
            priorityQ::PriorityQueue queue;
            node_map.clear();
            node_map.resize(node_cnt+1);
            visitedNodes.clear();
            visitedNodes.resize(node_cnt+1);
            // Create Parent Node
            djk_node* node = new djk_node;
            node->id = 0; // root
            node->depth = 0;
            node->index = 0;
            node->parent_id = 0;
            node->cost = 0;
            node->jt_config = ik_handler->init_guess;
            node_map[node->id] = node;
            queue.push(node->id,0,node->cost);
            visitedNodes[node->id] = true;
            std::cout<< "\nParent Node Initialized for " << i << " configuration\n";
            std::cout<< "The configuration is: " << ik_handler->init_guess.transpose()*180/M_PI << "\n\n";
            node_cnt = 0;
            while (!reach_goal && queue.size()!=0){
                int parent_id = queue.top();
                queue.pop();

                // Goal Check
                if (node_map[parent_id]->depth==wpTol_filtered.size()-1){ // Is it the last waypoint
                    if (node_map[parent_id]->cost<1e7){ // If IK was feasible at this goal node
                        std::cout<< "Reached Goal!!" << "\n";
                        std::cout<< "Depth: " << node_map[parent_id]->depth << ", Index: " << node_map[parent_id]->index << "\n"; 
                        reach_goal = true;
                        goal_id = parent_id;
                        std::cout<< "Max Depth Reached: " << node_map[parent_id]->depth << " | ";
                        std::cout<< "Nodes Evaluated: " << node_cnt << "\n";
                    }
                    continue;
                }


                // Debugging and progress monitoring
                if (node_map[parent_id]->depth>max_depth){
                    max_depth = node_map[parent_id]->depth;
                    std::cout<< "Max Depth Reached: " << max_depth << " | ";
                    std::cout<< "Nodes Evaluated: " << node_cnt << "\n";
                }

                // For all children i.e waypoints within tolerance
                // Expand each node and update cost
                Eigen::VectorXi children = WDG[node_map[parent_id]->depth][node_map[parent_id]->index];
                for (int j=0; j<children.size(); ++j){
                    // Get node id
                    int child_id = node_ids[node_map[parent_id]->depth+1][children(j)];
                    if (visitedNodes[child_id]){
                        ik_handler->init_guess = node_map[parent_id]->jt_config;
                        double g_cost;
                        if (ik_handler->solveIK(wpTol_filtered[node_map[parent_id]->depth+1].row(children(j)))) {
                            Eigen::MatrixXd jt_config = ik_handler->closest_sol;
                            std::vector<Eigen::MatrixXd> fk = ik_handler->robot->get_robot_FK_all_links( jt_config );
                            if (!wm->inCollision(fk)){
                                // g_cost = (solution.col(0)-node_map[parent_id]->jt_config).norm();
                                Eigen::ArrayXd jt_diff = (jt_config-node_map[parent_id]->jt_config);
                                g_cost = node_map[parent_id]->cost + jt_diff.abs().maxCoeff();
                                if (g_cost - node_map[child_id]->cost < 1e-7){ // If cost from this parent is less.
                                    node_map[child_id]->parent_id = parent_id; // Update Parent
                                    node_map[child_id]->cost = g_cost; // Update cost
                                    node_map[child_id]->jt_config = jt_config; // Update Configuration
                                    queue.push(node_map[child_id]->id, 0, node_map[child_id]->cost);
                                }
                            }
                            node_cnt++;
                        }
                        else{
                            continue; // From parent this node is not reachable but it is from other parent
                            node_cnt++;
                        }
                    }
                    else{
                        djk_node* node = new djk_node;
                        node->id = child_id; // root
                        node->depth = node_map[parent_id]->depth+1;
                        node->index = children(j);
                        node->parent_id = parent_id;
                        // Solve IK
                        ik_handler->init_guess = node_map[parent_id]->jt_config;
                        double g_cost;
                        if (ik_handler->solveIK(wpTol_filtered[node_map[parent_id]->depth+1].row(children(j)))){
                            Eigen::MatrixXd jt_config = ik_handler->closest_sol;
                            std::vector<Eigen::MatrixXd> fk = ik_handler->robot->get_robot_FK_all_links( jt_config );
                            if (!wm->inCollision(fk)){
                                node->jt_config = jt_config;
                                // g_cost = (solution.col(0)-node_map[parent_id]->jt_config).norm();
                                Eigen::ArrayXd jt_diff = (jt_config-node_map[parent_id]->jt_config);
                                g_cost = node_map[parent_id]->cost + jt_diff.abs().maxCoeff();
                                node->cost = g_cost;
                                visitedNodes[child_id] = true;
                                node_map[node->id] = node;
                                queue.push(node->id,0,node->cost);
                                node_cnt++;
                            }
                        }
                    }
                }

            }
            if (queue.size()==0 && !reach_goal){
                std::cout<< "Queue Empty. No Path Found.\n";
                std::cout<< "\n\n";
                continue;
            }
            
            std::cout<< "Number of Nodes Evaluations or IK calls: " << node_cnt << "\n";
            std::cout<< "Path Cost: " << node_map[goal_id]->cost << "\n";
            // Generate trajectory
            int id = goal_id;
            for(int row_no=wpTol.size()-1; row_no>-1; --row_no){
                curr_traj.row(row_no) = node_map[id]->jt_config;
                id = node_map[id]->parent_id;
            }
            if (node_map[goal_id]->cost - lowest_cost < 1e-7){
                std::cout<< "Found Path with lower Cost\n";
                lowest_cost = node_map[goal_id]->cost;
                trajectory = curr_traj;
            }
            // file_rw::file_write("/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv/" + std::to_string(i+1) + ".csv",curr_traj);
            std::cout<< "\n\n";
        }
        std::cout<< "Search Complete. Generating Trajectory.......\n";
        
        Eigen::MatrixXd success_flag;
        if (lowest_cost>1e7){ // Failure
            std::cout<< "SOLUTION NOT FOUND.......\n";
            success_flag = Eigen::MatrixXd::Ones(wpTol.size(),1)*0;
            return false;
        }
        else
            success_flag = Eigen::MatrixXd::Ones(wpTol.size(),1);

        ret_val.resize(wpTol.size(),ik_handler->OptVarDim+1);
        ret_val.block(0,0,wpTol.size(),ik_handler->OptVarDim) = trajectory;
        ret_val.block(0,ik_handler->OptVarDim,wpTol.size(),1) = success_flag;

        // Main Search
        return true;
    };    
};
#endif