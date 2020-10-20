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
        bool atleast_one_edge;
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
        atleast_one_edge = false;
        return atleast_one_edge;
    };

    void GreedyProgression(int strt, int end, std::vector<Eigen::MatrixXd>& ff_frames,
                        ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
                        std::vector<std::vector<int>>& unvisited_src, std::vector<node*>& node_map,
                        std::vector<Eigen::VectorXi>& node_list,
                        std::vector<Edge>& edges, std::vector<double>& weights, boost_graph* boost_graph
                        ){
        int a = 1;
        if (strt > end)
            a *= -1;
            
        Eigen::VectorXi prev_nodes;
        int depth = strt;
        bool isSource = true;
        // Iterate through each level from start to end
        while (depth >= 0 && depth <= ff_frames.size()-1){
            // Sample nodes to be added to graph
            Eigen::VectorXi sampled_nodes = sampler.GenNodeSamples(ff_frames, ik_handler, wm, 
                                geo_filter, unvisited_src, 
                             node_map, node_list, depth, isCreated, graph_metrics, isSource, boost_graph);
            if (sampled_nodes.size()==0){
                if (isSource)
                    if (unvisited_src[depth].size()!=0)
                        continue;
                return; // Couldn't go through all levels
            }
            // Integrate nodes with graph
            if (a==1){ // Fwd progression
                if (depth > 0)
                    MakeConnections( prev_nodes,sampled_nodes,edges, 
                        weights,ik_handler,node_map, boost_graph );
            }
            if (a==-1){ // Bckwd progression
                if (depth < ff_frames.size()-1)
                    MakeConnections( sampled_nodes,prev_nodes,edges, 
                        weights,ik_handler,node_map, boost_graph );
            }
            prev_nodes = sampled_nodes;
            depth += a;
            isSource = false;
        }
    };







public:
    int no_levels;
    // Vector to store graph performance metrics
    // Total number of edges, Total number of valid edges, Total number of nodes,
    // Total number of valid nodes, Total number of sources, Valid sources
    Eigen::VectorXi graph_metrics;
    // Cost tracker to track total edge cost for each level
    std::vector<double> cost_tracker;
    // Edge tracker to track number of edges for each level
    std::vector<int> edge_tracker;

    std::vector<std::vector<int>> unvisited_src;
    std::vector<Eigen::MatrixXi> isCreated;
    Eigen::VectorXi root_nodes;

    Actions(std::vector<Eigen::MatrixXd>& ff_frames){
        no_levels = ff_frames.size();
        graph_metrics = Eigen::VectorXi::Zero(6);
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
            isCreated[i] = Eigen::MatrixXi::Ones(ff_frames[i].rows(),8) * -1; // Since 8 solutions are possible
    };

    bool GreedyProgression(std::vector<Eigen::MatrixXd>& ff_frames,
                        ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
                        std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                        std::vector<Edge>& edges, std::vector<double>& weights, boost_graph* boost_graph){
        bool AreSamplesGen = false;
        if (unvisited_src[0].size()!=0){
            AreSamplesGen = true;
            GreedyProgression(0, ff_frames.size()-1, ff_frames,ik_handler,wm, geo_filter,
                            unvisited_src, node_map, node_list, edges, weights, boost_graph
                            );
        }
        if (unvisited_src[1].size()!=0){
            AreSamplesGen = true;
            GreedyProgression(ff_frames.size()-1, 0, ff_frames,ik_handler,wm, geo_filter,
                            unvisited_src, node_map, node_list, edges, weights, boost_graph
                            );
        }
        return AreSamplesGen;
    };

    bool InterConnections(ikHandler* ik_handler, std::vector<node*>& node_map, 
                        std::vector<Eigen::VectorXi>& node_list,
                        std::vector<Edge>& edges, std::vector<double>& weights, boost_graph* boost_graph){
        // Find the level with maximum cost from tracker and make interconnections
        // int max_index = std::distance(cost_tracker.begin(),
        //                     std::max_element( cost_tracker.begin(), cost_tracker.end() ));

        // Generate a random level index
        int index = 0 + ( std::rand() % ( no_levels ) );
        
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
        int depth = 1 + ( std::rand() % ( no_levels-1 ) );
        Eigen::VectorXi sampled_nodes = sampler.RandomSample(ff_frames, ik_handler, wm, geo_filter, 
                    node_map, node_list, depth, isCreated,
                    graph_metrics, boost_graph );
        if (sampled_nodes.size()==0)
            return false;
        if (node_list[depth-1].size()!=0)
            MakeConnections( node_list[depth-1],sampled_nodes,edges, 
                        weights,ik_handler,node_map, boost_graph );
        if (node_list[depth+1].size()!=0)
            MakeConnections( sampled_nodes, node_list[depth+1],edges, 
                        weights,ik_handler,node_map, boost_graph );

        return true;
    };
};


#endif