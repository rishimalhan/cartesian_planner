// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef __SS_SEARCHES__
#define __SS_SEARCHES__

#include <iostream>
#include <pct/numIK.hpp>
#include <Eigen/Eigen>
#include <queue>
#include <gen_utilities/priority_queue.hpp>
#include <ros/ros.h>
#include <gen_utilities/file_rw.hpp>

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
        // Eigen::VectorXd waypoint;
    };
    std::vector<bool> visitedNodes;
    std::vector<std::vector<int>> node_ids; // Assign node ids in increasing order to all waypoints
    std::vector<djk_node*> node_map;
public:
    ss_searches(){
        node_map.clear();
    };
    ~ss_searches(){};


    // djk_v1 (djikstra) uses one matrix to store visited nodes. Nodes are basically a struct.
    // The two matrices are initialized in the constructor when tolerance path and ik_handler is passed
    Eigen::MatrixXd djk_v1(numIK *ik_handler, std::vector<Eigen::MatrixXd> &wpTol){
        Eigen::MatrixXd trajectory(wpTol.size(),ik_handler->OptVarDim);
        // TODO: By default initial trajectory to home

        std::cout<< "\n\n###########################################################\n";
        // Find the first waypoint which is reachble. Terminate if none is
        Eigen::MatrixXd waypoints = wpTol[0];
        Eigen::MatrixXd reach_configs;
        Eigen::MatrixXi reach_ids; // depth, index
        std::cout<< "Number of starting waypoints: " << wpTol[0].rows() << "\n";
        int ctr = 0;
        // std::cout<< "Initial Guess: " << ik_handler->init_guess.transpose() << "\n";
        for (int i=0; i<waypoints.rows();++i){
            Eigen::MatrixXd solution = ik_handler->solveIK(waypoints.row(i));
            if (ik_handler->status){
                reach_configs.conservativeResize(ctr+1,ik_handler->OptVarDim);
                reach_ids.conservativeResize(ctr+1,2);
                reach_configs.row(ctr) = solution.col(0);
                reach_ids(ctr,0) = 0;
                reach_ids(ctr,1) = i;
                ctr++;
            }
        }
        if (reach_configs.rows()==0){
            std::cout<< "Start Point of the Path is Not Reachable\n";
            trajectory.conservativeResize(1,ik_handler->OptVarDim);
            return trajectory;
        }


        // Initialize Data Structures
        // Node id for i,j elements in wpTol can be accessed by using i and j
        std::cout<< "Initializing Djikstra Search.......\n";
        int node_cnt = 0;
        ctr = 0;
        node_ids.clear();
        
        for (int i=0; i<wpTol.size();++i){
            std::vector<int> temp1; temp1.clear();
            for (int j=0; j<wpTol[i].rows();++j){
                if (i==0){
                    // Assign root node value
                    temp1.push_back(ctr);
                    continue;
                } // root node
                else{
                    temp1.push_back(ctr);
                    ctr++;
                    node_cnt++;
                }
            }
            if (i==0)
                ctr++;
            node_ids.push_back(temp1);
        }
        

        std::cout<< "Number of Nodes Created: " << node_cnt << "\n";


        std::cout<< "Running Djikstra Search.......\n";
        bool reach_goal;
        // For each redundant configuration
        double lowest_cost = 10000000;
        int goal_id;
        // for (int i=0; i<reach_configs.rows();++i){
        for (int i=0; i<1;++i){
            Eigen::MatrixXd curr_traj(wpTol.size(),ik_handler->OptVarDim);
            int max_depth = 0;
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
            node->depth = reach_ids(i,0);
            node->index = reach_ids(i,1);
            node->parent_id = 0;
            node->cost = 0;
            node->jt_config = reach_configs.row(i).transpose();
            node_map[node->id] = node;
            queue.push(node->id,0,node->cost);
            visitedNodes[node->id] = true;
            std::cout<< "Parent Node Initialized for " << i+1 << " configuration\n";
            node_cnt = 0;
            while (!reach_goal){
                int parent_id = queue.top();
                queue.pop();
                int nxt_dpth = node_map[parent_id]->depth+1;

                // Goal Check
                if (node_map[parent_id]->depth==wpTol.size()-1){ // Is it the last waypoint
                    std::cout<< "Reached Goal!!" << "\n";
                    reach_goal = true;
                    goal_id = parent_id;
                    std::cout<< "Max Depth Reached: " << node_map[parent_id]->depth << " | ";
                    std::cout<< "Nodes Evaluated: " << node_cnt << "\n";
                    continue;
                }


                // Debugging and progress monitoring
                if (nxt_dpth>max_depth){
                    max_depth = nxt_dpth;
                    std::cout<< "Max Depth Reached: " << max_depth << " | ";
                    std::cout<< "Nodes Evaluated: " << node_cnt << "\n";
                }

                // std::cout<< "Next Depth: " << nxt_dpth << "\n";
                // For all children i.e waypoints within tolerance
                // Expand each node and update cost
                for (int j=0; j<wpTol[nxt_dpth].rows(); ++j){
                    // Get node id
                    int child_id = node_ids[nxt_dpth][j];
                    // If it is visited update cost else create a new one
                    // std::cout<< "isVisited: \n";
                    // std::cout<< visitedNodes[nxt_dpth][j] << "\n";
                    // std::cout<< "Printed" << "\n\n";
                    if (visitedNodes[child_id]){
                        ik_handler->init_guess = node_map[parent_id]->jt_config;
                        Eigen::MatrixXd solution = ik_handler->solveIK(wpTol[node_map[child_id]->depth].row(node_map[child_id]->index));
                        double g_cost;
                        if (ik_handler->status)
                            g_cost = (solution.col(0)-node_map[parent_id]->jt_config).norm();
                        else{
                            g_cost = 10000000;
                        }
                        if (g_cost - node_map[child_id]->cost < 1e-7){ // If cost from this parent is less.
                            node_map[child_id]->parent_id = parent_id; // Update Parent
                            node_map[child_id]->cost = node_map[parent_id]->cost + g_cost; // Update cost
                            queue.push(node_map[child_id]->id, 0, node_map[child_id]->cost);
                        }
                        node_cnt++;
                    }
                    else{
                        djk_node* node = new djk_node;
                        node->id = child_id; // root
                        node->depth = nxt_dpth;
                        node->index = j;
                        node->parent_id = parent_id;
                        // Solve IK
                        ik_handler->init_guess = node_map[parent_id]->jt_config;
                        Eigen::MatrixXd solution = ik_handler->solveIK(wpTol[node->depth].row(node->index));
                        double g_cost;
                        if (ik_handler->status)
                            g_cost = (solution.col(0)-node_map[parent_id]->jt_config).norm();
                        else{
                            g_cost = 10000000;
                        }
                        node->cost = node_map[parent_id]->cost + g_cost;
                        node->jt_config = solution.col(0);
                        visitedNodes[child_id] = true;
                        node_map[node->id] = node;
                        queue.push(node->id,0,node->cost);
                        node_cnt++;
                    }
                }

            }
            std::cout<< "Number of Nodes Evaluations or IK calls: " << node_cnt << "\n";
            std::cout<< "Path Cost: " << node_map[goal_id]->cost << "\n";
            // Generate trajectory
            int id = goal_id;
            for(int i=wpTol.size()-1; i!=0; --i){
                curr_traj.row(i) = node_map[id]->jt_config;
                id = node_map[id]->parent_id;
            }
            if (node_map[goal_id]->cost - lowest_cost < 1e-7){
                std::cout<< "Found Path with lower Cost\n\n";
                lowest_cost = node_map[goal_id]->cost;
                trajectory = curr_traj;
            }
            file_rw::file_write("/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv/" + std::to_string(i+1) + ".csv",curr_traj);
        }
        std::cout<< "Search Complete. Generating Trajectory.......\n";
        
        Eigen::MatrixXd success_flag;
        if (lowest_cost>1000000) // Failure
            success_flag = Eigen::MatrixXd::Ones(wpTol.size(),1)*0;
        else
            success_flag = Eigen::MatrixXd::Ones(wpTol.size(),1);

        Eigen::MatrixXd ret_val(wpTol.size(),ik_handler->OptVarDim+1);
        ret_val.block(0,0,wpTol.size(),ik_handler->OptVarDim) = trajectory;
        ret_val.block(0,ik_handler->OptVarDim,wpTol.size(),1) = success_flag;

        // Main Search
        return ret_val;
    };
    
};
#endif