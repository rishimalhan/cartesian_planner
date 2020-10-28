///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ACTION_MODULE__
#define __ACTION_MODULE__

#include <pct/node_description.hpp>
#include <random>
#include <Eigen/Eigen>
#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/transformation_utilities.hpp>
#include <pct/geometric_filter.h>
#include <pct/sample_nodes.hpp>
#include <pct/graph_description.hpp>
#include <unordered_set>
#include "nabo/nabo.h"

class Actions{
private:
    FFSampler sampler;
    


    bool isEdge(const std::vector<node*>& node_map, const int parent, const int child){
        if ((node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff() > 1.57)
            return false;
        return true;
    };

    double computeGCost( const std::vector<node*>& node_map, const int parent, const int child ){
        return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
        // return (node_map[child]->jt_config - node_map[parent]->jt_config).array().abs().maxCoeff();
        // return (node_map[child]->jt_config - node_map[parent]->jt_config).norm();
    };

    bool MakeConnections( Eigen::VectorXi parents, Eigen::VectorXi children, 
                    std::vector<Edge>& edges, std::vector<double>& weights,
                    ikHandler* ik_handler, const std::vector<node*>& node_map, boost_graph* boost_graph){
        bool atleast_one_edge = false;
        for (int i=0; i<parents.size(); ++i){
            for (int j=0; j<children.size(); ++j){
                graph_metrics(0)++;
                if ( isEdge(node_map, parents(i), children(j)) ){
                    if ( node_map[parents(i)]->depth==0 ){
                        bool add_root = true;
                        for (int k=0; k<root_nodes.size();++k)
                            if (root_nodes(k)==parents(i))
                                add_root = false;
                        if (add_root){
                            root_nodes.conservativeResize(root_nodes.size()+1);
                            root_nodes(root_nodes.size()-1) = parents(i);
                        }
                    }
                    graph_metrics(1)++;
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

    void GreedyProgression(int strt, int end, std::vector<Eigen::MatrixXd>& ff_frames,
                        ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
                        std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                        std::vector<Edge>& edges, std::vector<double>& weights, 
                        boost_graph* boost_graph, int& total_depth
                        ){
        int a = 1;
        if (strt > end)
            a *= -1;
            
        Eigen::VectorXi prev_nodes;
        int depth = strt;
        bool isSource = true;
        total_depth = 0;
        // Iterate through each level from start to end
        while (depth >= 0 && depth <= ff_frames.size()-1){
            // Sample nodes to be added to graph
            Eigen::VectorXi sampled_nodes = sampler.GenNodeSamples(ff_frames, ik_handler, wm, 
                                geo_filter, unvisited_src, 
                             node_map, node_list, depth, isCreated, graph_metrics, isSource, boost_graph);
            if (sampled_nodes.size()==0){
                if (isSource)
                    if (depth==0)
                        if (unvisited_src[depth].size()!=0)
                            continue;
                    if (depth==no_levels-1)
                        if (unvisited_src[1].size()!=0)
                            continue;
                return; // Couldn't go through all levels
            }
            // Integrate nodes with graph
            if (a==1){ // Fwd progression
                if (depth > 0)
                    if (!MakeConnections( prev_nodes,sampled_nodes,edges, 
                        weights,ik_handler,node_map, boost_graph ))
                        return; // Couldn't go through all levels
            }
            if (a==-1){ // Bckwd progression
                if (depth < ff_frames.size()-1)
                    if (!MakeConnections( sampled_nodes,prev_nodes,edges, 
                        weights,ik_handler,node_map, boost_graph ))
                        return; // Couldn't go through all levels
            }
            prev_nodes = sampled_nodes;
            depth += a;
            isSource = false;
            total_depth++;
        }
    };







public:
    int infeasibility;
    int no_levels;
    // Vector to store graph performance metrics
    // 0: Total number of edges
    // 1: Total number of valid edges
    // 2: Total number of nodes
    // 3: Total number of valid nodes
    // 4: Total number of fwd sources
    // 5: Valid fwd sources
    // 6: Avg fwd depth
    // 7: Max fwd depth
    // 8: Total fwd depth
    // 9: Total number of bck sources
    // 10: Valid bck sources
    // 11: Avg bck depth
    // 12: Max bck depth
    // 13: Total bck depth

    Eigen::VectorXd graph_metrics;
    // Cost tracker to track total edge cost for each level
    std::vector<double> cost_tracker;
    // Edge tracker to track number of edges for each level
    std::vector<int> edge_tracker;

    std::vector<std::vector<int>> unvisited_src;
    std::vector<Eigen::MatrixXi> isCreated;
    Eigen::VectorXi root_nodes;

    Actions(std::vector<Eigen::MatrixXd>& ff_frames){
        infeasibility = sampler.infeasibility;
        no_levels = ff_frames.size();
        sampler.M.resize(no_levels);
        // Nabo initializer
        sampler.kdtrees.resize(no_levels);

        for (int i=0; i<no_levels; ++i){
            sampler.M[i].resize(7,ff_frames[i].rows());
            for (int j=0; j<ff_frames[i].rows(); ++j)
                sampler.M[i].col(j) = sampler.GetQTWp( ff_frames[i].row(j).transpose() ).cast<float>();
            sampler.kdtrees[i] = Nabo::NNSearchF::createKDTreeLinearHeap(sampler.M[i]);
        }
        

        graph_metrics = Eigen::VectorXd::Zero(14);
        cost_tracker.clear();
        edge_tracker.clear();
        cost_tracker.resize(ff_frames.size());
        edge_tracker.resize(ff_frames.size());
        unvisited_src.resize(2);
        for (int i=0; i<ff_frames.size(); ++i){
            cost_tracker[i] = 0;
            edge_tracker[i] = 0;
        }
        for (int i=0; i<ff_frames[0].rows(); ++i)
            unvisited_src[0].push_back(i);
        for (int i=0; i<ff_frames[ff_frames.size()-1].rows(); ++i)
            unvisited_src[1].push_back(i);
        // List to store the created node references 
        isCreated.clear();
        isCreated.resize(ff_frames.size());
        for (int i=0; i<ff_frames.size(); ++i)
            // Since 8 solutions are possible. First Column is if waypoint is reachable
            isCreated[i] = Eigen::MatrixXi::Ones(ff_frames[i].rows(),9) * -1; 
    };

    ~Actions(){// cleanup kd-tree
        for (int i=0; i<no_levels; ++i)
            delete sampler.kdtrees[i];
    }

    bool GreedyProgression(std::vector<Eigen::MatrixXd>& ff_frames,
                        ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
                        std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                        std::vector<Edge>& edges, std::vector<double>& weights, boost_graph* boost_graph,
                        string type){
        bool AreSamplesGen = false;
        int total_depth;
        if (type=="fwd"){
            if (unvisited_src[0].size()!=0){
                AreSamplesGen = true;
                GreedyProgression(0, ff_frames.size()-1, ff_frames,ik_handler,wm, geo_filter,
                                node_map, node_list, edges, weights, boost_graph, total_depth
                                );
                // Avg Fwd Depth
                graph_metrics(6) = ((graph_metrics(5)-1)*graph_metrics(6) 
                                + total_depth)/graph_metrics(5);
                // Max Fwd Depth
                if (total_depth > graph_metrics(7))
                    graph_metrics(7) = total_depth;
            }
        }
        if (type=="bck"){
            if (unvisited_src[1].size()!=0){
                AreSamplesGen = true;
                GreedyProgression(ff_frames.size()-1, 0, ff_frames,ik_handler,wm, geo_filter,
                                node_map, node_list, edges, weights, boost_graph, total_depth
                                );
                // Avg Bck Depth
                graph_metrics(11) = ((graph_metrics(10)-1)*graph_metrics(11) 
                                + total_depth)/graph_metrics(10);
                // Max Bck Depth
                if (total_depth > graph_metrics(12))
                    graph_metrics(12) = total_depth;
            }
        }
        infeasibility = sampler.infeasibility;
        return AreSamplesGen;
    };

    bool InterConnections(ikHandler* ik_handler, std::vector<node*>& node_map, 
                        std::vector<Eigen::VectorXi>& node_list,
                        std::vector<Edge>& edges, std::vector<double>& weights, boost_graph* boost_graph,
                        std::vector<double>& path_costs){
        // Find the level with maximum cost from tracker and make interconnections
        // Create Bias
        int index;
        if ( std::accumulate(path_costs.begin(),path_costs.end(),0) > 1e-5 )
            index = std::distance(path_costs.begin(),
                                std::max_element( path_costs.begin(), path_costs.end() ));
        else
            // Generate a random level index
            index = 0 + ( std::rand() % ( no_levels ) );
        
        // Connection with nodes above
        if (index-1 >= 0){
            MakeConnections( node_list[index-1],node_list[index],edges, 
                        weights,ik_handler,node_map, boost_graph );
        }

        if (index+1 <= no_levels-1){
            MakeConnections( node_list[index],node_list[index+1],edges, 
                        weights,ik_handler,node_map, boost_graph );
        }

        return true;
    };

    bool NodeAdditions(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list, boost_graph* boost_graph,
                    std::vector<Edge>& edges, std::vector<double>& weights){
        // Generate a random level index
        int last_index = no_levels-2;
        int depth = 1 + ( std::rand() % ( last_index ) );
        sampler.RandomSample(ff_frames, ik_handler, wm, geo_filter, 
                    node_map, node_list, depth, isCreated,
                    graph_metrics, boost_graph );
        // if (sampled_nodes.size()==0)
        //     return false;
        // if (node_list[depth-1].size()!=0)
        //     MakeConnections( node_list[depth-1],sampled_nodes,edges, 
        //                 weights,ik_handler,node_map, boost_graph );
        // if (node_list[depth+1].size()!=0)
        //     MakeConnections( sampled_nodes, node_list[depth+1],edges, 
        //                 weights,ik_handler,node_map, boost_graph );

        return true;
    };
};


#endif