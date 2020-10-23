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
Eigen::VectorXd prev_wp;
int prev_index;
int prev_depth;

node* generate_node(Eigen::VectorXd joint_cfg,
                    int node_id, int node_depth, Eigen::VectorXd waypoint,
                    ikHandler* ik_handler){
    
    node* new_node = new node;
    new_node->id = node_id;
    new_node->jt_config = joint_cfg;
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


bool GreedySamples(std::vector<std::vector<int>>& unvisited_src,
                    std::vector<Eigen::MatrixXi>& isCreated,
                    std::vector<Eigen::MatrixXd>& ff_frames, int depth, Eigen::VectorXd& waypoint,
                    int& optID, Eigen::VectorXi& graph_metrics, bool isSource, int trial_itr ){
    if (isSource){
        int src_id;
        if (depth==0)
            src_id = 0;
        else
            src_id = 1;
        // Generate a random ff_frame index
        int index = 0 + ( std::rand() % ( unvisited_src[src_id].size() ) );
        auto ff_frame_id = unvisited_src[src_id][index];
        vector<int>::iterator it = unvisited_src[src_id].begin() + index;
        unvisited_src[src_id].erase( it );
        // Get the corresponding waypoint
        waypoint = ff_frames[depth].row(ff_frame_id).transpose();
        graph_metrics(4)++;
        optID = ff_frame_id;
        // Check if a node has already been created
        if (isCreated[depth](optID,0) != -1)
            return false;
        return true;
    }
    
    // To speed up add an option to remove element from kdtree

    // OptID defined here
    Eigen::VectorXi indices(trial_itr+1);
    Eigen::VectorXf dists2(trial_itr+1);
    kdtrees[depth]->knn(prev_wp.cast<float>(), indices, dists2, trial_itr+1);
    optID = indices(trial_itr);

    waypoint = ff_frames[depth].row(optID).transpose();
    // Check if a node has already been created
    if (isCreated[depth](optID,0) != -1)
        return false;
    return true;
};

public:
std::vector<Nabo::NNSearchF*> kdtrees;
std::vector<Eigen::MatrixXf> M;
Eigen::MatrixXi graph_stats;
bool infeasibility;
int max_samples;
FFSampler(){infeasibility = false;};
~FFSampler(){};

Eigen::VectorXd GetQTWp(Eigen::VectorXd waypoint){
    Eigen::VectorXd wp_qt(7);
    wp_qt.segment(0,3) = waypoint.segment(0,3);
    wp_qt.segment(3,4) = rtf::bxbybz2qt(waypoint.segment(3,9).transpose()).row(0).transpose();
    return wp_qt;
};


Eigen::VectorXi GenNodeSamples(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter,
                    std::vector<std::vector<int>>& unvisited_src, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list,  
                    int depth, std::vector<Eigen::MatrixXi>& isCreated, 
                    Eigen::VectorXi& graph_metrics, bool isSource, boost_graph* boost_graph){
    int optID;
    Eigen::VectorXi sampled_nodes;
    Eigen::VectorXd waypoint;
    int trial_itr = 0;
    while (trial_itr < ff_frames[depth].rows()){
        if(!GreedySamples(unvisited_src,isCreated, ff_frames,depth,waypoint,
                            optID, graph_metrics, isSource, trial_itr)){
            prev_wp = GetQTWp(waypoint);
            prev_index = optID;
            prev_depth = depth;
            // Node already exists so we return the existing node ids
            for (int i=0; i<isCreated[depth].cols(); ++i){
                if ( isCreated[depth](optID,i)!=-1 ){
                    sampled_nodes.conservativeResize(sampled_nodes.size()+1);
                    sampled_nodes(sampled_nodes.size()-1) = isCreated[depth](optID,i);
                }
            }
            return sampled_nodes;
        }
        prev_wp = GetQTWp(waypoint);
        prev_index = optID;
        prev_depth = depth;
        // Solve IK and check for collision
        // If valid solution, then create a node out of it
        int no_sols = 0;
        if ( ik_handler->solveIK(waypoint) ){
            // For every solution create a node
            for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
                graph_metrics(2)++;
                // Check for collision
                std::vector<Eigen::MatrixXd> fk_kdl = 
                ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                if (geo_filter->is_tool_collision_free_(waypoint)){
                    if(!wm->inCollision( fk_kdl )){
                        graph_metrics(3)++;
                        if (isSource)
                            graph_metrics(5)++;
                        int node_id = node_map.size();
                        node* new_node = generate_node(ik_handler->solution.col(sol_no), 
                                            node_id, depth, waypoint,ik_handler);
                        node_map.push_back(new_node);
                        sampled_nodes.conservativeResize(sampled_nodes.size()+1);
                        sampled_nodes(sampled_nodes.size()-1) = node_id;
                        node_list[depth].conservativeResize(node_list[depth].size()+1);
                        node_list[depth](node_list[depth].size()-1) = node_id;
                        isCreated[depth](optID,sol_no) = node_id;
                        boost_graph->p.push_back(vertex(node_id,boost_graph->g));
                    }
                }
            }
        }
        if (isSource)
            return sampled_nodes;
        if (sampled_nodes.size()!=0)
            return sampled_nodes;
        trial_itr++;
    }
    if (sampled_nodes.size()==0)
        infeasibility = true;
    return sampled_nodes;
};


bool RandomSample(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list,  
                    int depth, std::vector<Eigen::MatrixXi>& isCreated,
                    Eigen::VectorXi& graph_metrics, boost_graph* boost_graph){
    Eigen::VectorXi sampled_nodes;
    Eigen::VectorXd waypoint;
    std::vector<int> unvisited_samples; unvisited_samples.clear();
    for (int i=0; i<ff_frames[depth].rows();++i)
        unvisited_samples.push_back(i);

    while (unvisited_samples.size()!=0){
        // Generate a random ff_frame index
        int index = 0 + ( std::rand() % ( unvisited_samples.size() ) );
        auto ff_frame_id = unvisited_samples[index];
        vector<int>::iterator it = unvisited_samples.begin() + index;
        unvisited_samples.erase( it );
        if (isCreated[depth](ff_frame_id,0)!=-1)
            continue;
        waypoint = ff_frames[depth].row(ff_frame_id).transpose();

        // Solve IK and check for collision
        // If valid solution, then create a node out of it
        int no_sols = 0;
        if ( ik_handler->solveIK(waypoint) ){
            // For every solution create a node
            for (int sol_no=0; sol_no<ik_handler->solution.cols();++sol_no){
                graph_metrics(2)++;
                // Check for collision
                std::vector<Eigen::MatrixXd> fk_kdl = 
                ik_handler->robot->get_robot_FK_all_links(ik_handler->solution.col(sol_no));
                if (geo_filter->is_tool_collision_free_(waypoint)){
                    if(!wm->inCollision( fk_kdl )){
                        graph_metrics(3)++;
                        int node_id = node_map.size();
                        node* new_node = generate_node(ik_handler->solution.col(sol_no), 
                                            node_id, depth, waypoint,ik_handler);
                        node_map.push_back(new_node);
                        sampled_nodes.conservativeResize(sampled_nodes.size()+1);
                        sampled_nodes(sampled_nodes.size()-1) = node_id;
                        node_list[depth].conservativeResize(node_list[depth].size()+1);
                        node_list[depth](node_list[depth].size()-1) = node_id;
                        isCreated[depth](ff_frame_id,sol_no) = node_id;
                        boost_graph->p.push_back(vertex(node_id,boost_graph->g));
                    }
                }
            }
        }
        if (sampled_nodes.size()==0)
            continue;
    }
    return true;
};

};
#endif