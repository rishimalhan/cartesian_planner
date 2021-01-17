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
#include <stack>

class Actions{
private:
    // bool MakeConnections( Eigen::VectorXi parents, Eigen::VectorXi children, 
    //                 ikHandler* ik_handler, const std::vector<node*>& node_map, boost_graph* boost_graph){
    //     bool atleast_one_edge = false;
    //     if (parents.size()==0 || children.size()==0)
    //         return atleast_one_edge;

    //     // Connect dummy root
    //     if (node_map[parents(0)]->depth==0){
    //         boost_graph->root_connected = true;
    //         for (int i=0; i<parents.size(); ++i)
    //             add_edge( boost_graph->p[0], boost_graph->p[parents(i)],
    //                         EdgeWeightProperty(0),boost_graph->g );
    //     }
    //     if (node_map[parents(0)]->depth==boost_graph->no_levels-1){
    //         boost_graph->root_connected = true;
    //         for (int i=0; i<parents.size(); ++i)
    //             add_edge( boost_graph->p[parents(i)], boost_graph->p[1],
    //                         EdgeWeightProperty(0),boost_graph->g );
    //     }
        
    //     // Connect dummy leaf
    //     if (node_map[children(0)]->depth==boost_graph->no_levels-1){
    //         boost_graph->leaf_connected = true;
    //         for (int j=0; j<children.size(); ++j)
    //             add_edge( boost_graph->p[children(j)], boost_graph->p[1],
    //                         EdgeWeightProperty(0),boost_graph->g );
    //     }
    //     if (node_map[children(0)]->depth==0){
    //         boost_graph->leaf_connected = true;
    //         for (int j=0; j<children.size(); ++j)
    //             add_edge( boost_graph->p[0], boost_graph->p[children(j)],
    //                         EdgeWeightProperty(0),boost_graph->g );
    //     }
            
    //     for (int i=0; i<parents.size(); ++i){
    //         for (int j=0; j<children.size(); ++j){
    //             // graph_metrics(0)++;
    //             if ( isEdge(node_map, parents(i), children(j)) ){
    //                 // graph_metrics(1)++;
    //                 // edges.push_back( Edge(parents(i),children(j)) );
    //                 double cost = computeGCost(node_map, parents(i), children(j));
    //                 // weights.push_back( cost );
    //                 add_edge( boost_graph->p[parents(i)],
    //                             boost_graph->p[children(j)],EdgeWeightProperty(cost),boost_graph->g );
    //                 atleast_one_edge = true;
    //             }
    //         }
    //     }
    //     return atleast_one_edge;
    // };

    // void GreedyProgression(int strt, int end, std::vector<Eigen::MatrixXd>& ff_frames,
    //                     ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
    //                     std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
    //                     boost_graph* boost_graph, int& total_depth, bool src_bias, int resource
    //                     ){
    //     int a = 1;
    //     if (strt > end)
    //         a *= -1;
            
    //     Eigen::VectorXi prev_nodes;
    //     int depth = strt;
    //     bool isSource = true;
    //     total_depth = 0;
    //     // Iterate through each level from start to end
    //     while (depth >= 0 && depth <= ff_frames.size()-1){
    //         // Sample nodes to be added to graph
    //         Eigen::VectorXi sampled_nodes = sampler.GenNodeSamples(ff_frames, ik_handler, wm, 
    //                             geo_filter, unvisited_src, node_map, node_list, depth, 
    //                             isCreated, graph_metrics, isSource, boost_graph, src_bias, resource);
    //         if (sampled_nodes.size()==0){
    //             if (isSource)
    //                 if (depth==0)
    //                     if (unvisited_src[depth].size()!=0)
    //                         continue;
    //                 if (depth==no_levels-1)
    //                     if (unvisited_src[1].size()!=0)
    //                         continue;
    //             return; // Couldn't go through all levels
    //         }
            
    //         // Integrate nodes with graph
    //         if (a==1){ // Fwd progression
    //             if (depth > 0)
    //                 if (!MakeConnections( prev_nodes,sampled_nodes,ik_handler,node_map, boost_graph ))
    //                     return; // Couldn't go through all levels
    //         }
    //         if (a==-1){ // Bckwd progression
    //             if (depth < ff_frames.size()-1)
    //                 if (!MakeConnections( sampled_nodes,prev_nodes,ik_handler,node_map, boost_graph ))
    //                     return; // Couldn't go through all levels
    //         }
    //         prev_nodes = sampled_nodes;
    //         depth += a;
    //         isSource = false;
    //         total_depth++;
    //     }
    // };


    bool InitSource(int seq, std::vector<Eigen::MatrixXd>& ff_frames,
                        ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
                        std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                        boost_graph* boost_graph, bool src_bias, int resource, 
                        std::stack<std::vector<int>>& greedy_ff){
        int depth;
        int src_id;
        Eigen::MatrixXi sampled_wps;
        if (seq==1){
            depth = 0;
            src_id = 0;
        }
        if (seq==-1){
            depth = ff_frames.size()-1;
            src_id = 1;
        }
        bool sourceFound = false;
        Eigen::VectorXd src_wp;
        int unv_id;
        while(!sourceFound){
            bool samples_found = sampler.GenNodeSamples(ff_frames, ik_handler, wm, 
                                geo_filter, unvisited_src, node_map, node_list, depth, sampled_wps,
                                isCreated, graph_metrics, true, boost_graph, src_bias, src_wp, unv_id);
            if (src_wp.size()!=0){
                sampler.src_balls[src_id].conservativeResize(sampler.src_balls[src_id].size()+1);
                sampler.src_costs[src_id].conservativeResize(sampler.src_costs[src_id].size()+1);

                if (src_id==0){
                    sampler.src_waypoints.conservativeResize(7,sampler.src_waypoints.cols()+1);
                    sampler.src_waypoints.col(sampler.src_waypoints.cols()-1) = src_wp;
                }

                if (src_id==1){
                    sampler.snk_waypoints.conservativeResize(7,sampler.snk_waypoints.cols()+1);
                    sampler.snk_waypoints.col(sampler.snk_waypoints.cols()-1) = src_wp;
                }

                sampler.src_ids[src_id].conservativeResize(sampler.src_ids[src_id].size()+1);
                sampler.src_ids[src_id](sampler.src_ids[src_id].size()-1) = unv_id;
            }

            if (!samples_found){
                sampler.src_costs[src_id](sampler.src_costs[src_id].size()-1) = 1e8;
                if (depth==0)
                    if (unvisited_src[0].size()!=0)
                        continue;
                if (depth==no_levels-1)
                    if (unvisited_src[1].size()!=0)
                        continue;
                return false; // No source found
            }
            sourceFound = true;
        }
        std::vector<int> ff_node = {depth,sampled_wps(0,0)};
        greedy_ff.push( ff_node );
        greedy_list[depth] = sampled_wps.block(0,1,1,8);
        // if (src_id==0)
        //     ROS_WARN_STREAM("Picking FWD SRC: " << sampled_wps(0,0));
        // if (src_id==1)
        //     ROS_WARN_STREAM("Picking BCK SRC: " << sampled_wps(0,0));
        return true;
    }


    void DFSProgression(int seq, std::vector<Eigen::MatrixXd>& ff_frames,
                        ikHandler* ik_handler, WM::WM* wm, GeometricFilterHarness* geo_filter,
                        std::vector<node*>& node_map, std::vector<Eigen::VectorXi>& node_list,
                        boost_graph* boost_graph, bool src_bias, std::stack<std::vector<int>> greedy_ff
                        ){
        std::vector<std::unordered_set<int>> visited_ids;
        visited_ids.resize(ff_frames.size());
        // If stack empty we need a new source
        while (!greedy_ff.empty()){
            int depth;
            Eigen::MatrixXi sampled_wps;
            std::vector<int> ff_node(2);
            
            ff_node = greedy_ff.top();
            sampler.prev_wp = sampler.GetQTWp( ff_frames[ff_node[0]].row(ff_node[1]).transpose() );
            depth = ff_node[0] + seq;
            greedy_ff.pop();
            if (depth<=ff_frames.size()-1 && depth>=0){
                bool samples_found = false;
                Eigen::VectorXd src_wp;
                int unv_id;
                while (!samples_found){
                    // Picking next nodes
                    samples_found = sampler.GenNodeSamples(ff_frames, ik_handler, wm, 
                                geo_filter, unvisited_src, node_map, node_list, depth, sampled_wps,
                                isCreated, graph_metrics, false, boost_graph, src_bias, src_wp, unv_id);
                }

                Eigen::MatrixXi sampled_wp_list;
                for (int i=0; i<sampled_wps.rows(); ++i){
                    ff_node = {depth,sampled_wps(i,0)};
                    int old_size = visited_ids[depth].size();
                    visited_ids[depth].insert(sampled_wps(i,0));
                    if (visited_ids[depth].size()-old_size!=0){
                        greedy_ff.push( ff_node );
                        sampled_wp_list.conservativeResize(sampled_wp_list.rows()+1,sampled_wps.cols()-1);
                        sampled_wp_list.row(sampled_wp_list.rows()-1) = sampled_wps.block(i,1,1,8);
                    }
                }

                if (sampled_wp_list.rows()>0){
                    if (greedy_list[depth].rows()==0)
                        greedy_list[depth] = sampled_wp_list;
                    else{
                        Eigen::MatrixXi combined_mat(greedy_list[depth].rows()+sampled_wp_list.rows(),
                                                    sampled_wp_list.cols());
                        combined_mat << greedy_list[depth] , sampled_wp_list;
                        greedy_list[depth] = combined_mat;
                    }
                }
            }
        }
    };







public:
    FFSampler sampler;
    bool feasibility;
    int no_levels;
    double ball_size;
    // // Find the following for each level
    // // Number of BC
    // Eigen::VectorXd bc_count;
    // // Maintain truth values of LC and UC
    // Eigen::MatrixXi truth_mat;
    // // Min cost to go to next level for a level
    // Eigen::VectorXd min_transition_cost;

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
    // std::vector<double> cost_tracker;
    // Edge tracker to track number of edges for each level
    // std::vector<int> edge_tracker;

    std::vector<std::vector<int>> unvisited_src;
    std::vector<Eigen::MatrixXi> isCreated;
    Eigen::VectorXi root_nodes;
    std::vector<Eigen::MatrixXi> greedy_list;


    Actions(std::vector<Eigen::MatrixXd>& ff_frames){
        feasibility = true;
        no_levels = ff_frames.size();
        sampler.M.resize(no_levels);
        ball_size = 0.05;
        // bc_count = Eigen::VectorXd::Zero(no_levels); // UC, LC, BC
        // bc_count(0) = 1e8;
        // bc_count(no_levels-1) = 1e8;
        // min_transition_cost = Eigen::VectorXd::Ones(no_levels)*1e8;

        // Nabo initializer
        sampler.kdtrees.resize(no_levels);

        for (int i=0; i<no_levels; ++i){
            sampler.M[i].resize(7,ff_frames[i].rows());
            // sampler.M[i].resize(3,ff_frames[i].rows());
            for (int j=0; j<ff_frames[i].rows(); ++j)
                sampler.M[i].col(j) = sampler.GetQTWp( ff_frames[i].row(j).transpose() );
                // sampler.M[i].col(j) = ff_frames[i].block(j,0,1,3).transpose();
            sampler.kdtrees[i] = Nabo::NNSearchD::createKDTreeLinearHeap(sampler.M[i]);
        }

        graph_metrics = Eigen::VectorXd::Zero(14);
        // cost_tracker.clear();
        // edge_tracker.clear();
        // cost_tracker.resize(ff_frames.size());
        // edge_tracker.resize(ff_frames.size());
        unvisited_src.resize(2);
        // for (int i=0; i<ff_frames.size(); ++i){
        //     cost_tracker[i] = 0;
        //     edge_tracker[i] = 0;
        // }
        for (int i=0; i<ff_frames[0].rows(); ++i)
            unvisited_src[0].push_back(i);
        for (int i=0; i<ff_frames[ff_frames.size()-1].rows(); ++i)
            unvisited_src[1].push_back(i);

        auto rng = std::default_random_engine {};
        std::shuffle(std::begin(unvisited_src[0]), std::end(unvisited_src[0]), rng);
        std::shuffle(std::begin(unvisited_src[1]), std::end(unvisited_src[1]), rng);

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
                        boost_graph* boost_graph, string type, bool src_bias, int resource){
        if (unvisited_src[0].size()==0 && sampler.junk_yard[0].size()!=0){
            ROS_WARN_STREAM("Restoring fwd nodes");
            unvisited_src[0] = sampler.junk_yard[0];
            sampler.junk_yard[0].clear();
            sampler.reject_samples = false;
        }

        if (unvisited_src[1].size()==0 && sampler.junk_yard[1].size()!=0){
            ROS_WARN_STREAM("Restoring bck nodes");
            unvisited_src[1] = sampler.junk_yard[1];
            sampler.junk_yard[1].clear();
            sampler.reject_samples = false;
        }

        std::stack<std::vector<int>> greedy_ff;
        greedy_list.clear();
        greedy_list.resize(ff_frames.size());
        int total_depth;
        if (type=="fwd"){
            if (unvisited_src[0].size()!=0){
                if ( InitSource(1, ff_frames, ik_handler, wm, geo_filter, node_map, node_list,
                        boost_graph, src_bias, resource, greedy_ff) )
                    DFSProgression( 1, ff_frames,ik_handler,wm, geo_filter, node_map, node_list, 
                        boost_graph, src_bias, greedy_ff );
                // // Avg Fwd Depth
                // graph_metrics(6) = ((graph_metrics(5)-1)*graph_metrics(6) 
                //                 + total_depth)/graph_metrics(5);
                // // Max Fwd Depth
                // if (total_depth > graph_metrics(7))
                //     graph_metrics(7) = total_depth;
            }
        }
        if (type=="bck"){
            if (unvisited_src[1].size()!=0){
                if ( InitSource(-1, ff_frames, ik_handler, wm, geo_filter, node_map, node_list,
                        boost_graph, src_bias, resource, greedy_ff) )
                    DFSProgression( -1, ff_frames,ik_handler,wm, geo_filter, node_map, node_list, 
                        boost_graph, src_bias, greedy_ff );
                // // Avg Bck Depth
                // graph_metrics(11) = ((graph_metrics(10)-1)*graph_metrics(11) 
                //                 + total_depth)/graph_metrics(10);
                // // Max Bck Depth
                // if (total_depth > graph_metrics(12))
                //     graph_metrics(12) = total_depth;
            }
        }

        if (unvisited_src[0].size()==0 && sampler.junk_yard[0].size()!=0){
            ROS_WARN_STREAM("Restoring fwd nodes");
            unvisited_src[0] = sampler.junk_yard[0];
            sampler.junk_yard[0].clear();
            sampler.reject_samples = false;
        }

        if (unvisited_src[1].size()==0 && sampler.junk_yard[1].size()!=0){
            ROS_WARN_STREAM("Restoring bck nodes");
            unvisited_src[1] = sampler.junk_yard[1];
            sampler.junk_yard[1].clear();
            sampler.reject_samples = false;
        }


        if (unvisited_src[0].size()==0 && unvisited_src[1].size()==0)
            feasibility = false;
        return feasibility;
    };

    void sort_vec(const Eigen::VectorXd& vec, Eigen::VectorXd& sorted_vec,  
                        Eigen::VectorXi& ind){

        // Sort Descending
        ind=Eigen::VectorXi::LinSpaced(vec.size(),0,vec.size()-1);//[0 1 2 3 ... N-1]
        auto rule=[vec](int i, int j)->bool{
            return vec(i)>vec(j);
        };// regular expression, as a predicate of sort
        std::sort(ind.data(),ind.data()+ind.size(),rule);
        //The data member function returns a pointer to the first element of VectorXd, similar to begin()
        sorted_vec.resize(vec.size());
        for(int i=0;i<vec.size();i++){
            sorted_vec(i)=vec(ind(i));
}
    };

    double closest(std::vector<double> const& vec, double value) {
        auto const it = std::lower_bound(vec.begin(), vec.end(), value);
        if (it == vec.end()) { return -1; }

        return it - vec.begin();
    }

    // bool InterConnections(ikHandler* ik_handler, std::vector<node*>& node_map, 
    //                     std::vector<Eigen::VectorXi>& node_list,
    //                     std::vector<Edge>& edges, std::vector<double>& weights, boost_graph* boost_graph,
    //                     double path_cost, std::vector<double> path_costs){
    //     // Find the level with maximum cost from tracker and make interconnections
    //     int index;
    //     // Create Bias
    //     // Eigen::VectorXd node_counts(node_list.size());
    //     // for (int i=0; i<node_list.size(); ++i)
    //     //     node_counts(i) = node_list[i].size();

    //     // Eigen::VectorXd temp1 = bc_count.array().inverse();
    //     // temp1 = (temp1 / temp1.maxCoeff()) * 1;
    //     // Eigen::VectorXd temp2 = node_counts.array().inverse();
    //     // temp2 = (temp2 / temp2.maxCoeff()) * 0;
    //     // Eigen::VectorXd score = temp1 + temp2;

    //     // Eigen::VectorXd sorted_conn;
    //     // Eigen::VectorXi sorted_index;
    //     // sort_vec(score,sorted_conn,sorted_index);
    //     // // Create probabilities for top 4 bottlenecks
    //     // Eigen::VectorXd probs(5);
    //     // probs(4) = 0.2; // Prob for randomly sampling a depth
    //     // probs.segment(0,4) = sorted_conn.segment(0,4) / 
    //     //                     (sorted_conn(0)+sorted_conn(1)+sorted_conn(2)+sorted_conn(3));
    //     // probs.segment(0,4) *= (1-probs(4));
        
    //     // Eigen::VectorXi depths(5);
    //     // depths.segment(0,4) = sorted_index.segment(0,4);
    //     // // Generate a random level index
    //     // depths(4) = 0 + ( std::rand() % ( no_levels ) );

    //     // double prob = (double) std::rand() / RAND_MAX;
    //     // if (prob <= probs(0))
    //     //     index = depths(0);
    //     // if (prob <= probs(0)+probs(1) && prob > probs(0))
    //     //     index = depths(1);
    //     // if (prob <= probs(0)+probs(1)+probs(2) && prob > probs(0)+probs(1))
    //     //     index = depths(2);
    //     // if (prob <= probs(0)+probs(1)+probs(2)+probs(3) && prob > probs(0)+probs(1)+probs(2))
    //     //     index = depths(3);
    //     // if (prob <= 1.0 && prob > probs(0)+probs(1)+probs(2)+probs(3))
    //     //     index = depths(4);

    //     // ROS_INFO_STREAM("Top Candidates: " << depths.transpose() << "\n");

    //     index = 0 + ( std::rand() % ( no_levels ) );
    //     // Connection with nodes above
    //     if (index-1 >= 0){
    //         MakeConnections( node_list[index-1],node_list[index],ik_handler,node_map, boost_graph );
    //     }

    //     if (index+1 <= no_levels-1){
    //         MakeConnections( node_list[index],node_list[index+1],ik_handler,node_map, boost_graph );
    //     }

    //     return true;
    // };

    // bool EdgeConnections(ikHandler* ik_handler, std::vector<Eigen::VectorXi>& node_list,
    //                 boost_graph* boost_graph, std::vector<node*>& node_map){
    //     for (int index=0; index<node_list.size()-1; ++index)
    //         MakeConnections( node_list[index],node_list[index+1],ik_handler,node_map, boost_graph );
    //     return true;
    // };

    void NearestNode(ikHandler* ik_handler, WM::WM* wm, Eigen::VectorXd waypoint,
                std::vector<Eigen::MatrixXd>& ff_frames, int depth,
                boost_graph* boost_graph,
                GeometricFilterHarness* geo_filter, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list, double resource){
        sampler.NearestNode(ik_handler, wm, waypoint,
                ff_frames, depth, isCreated, boost_graph, geo_filter,
                node_map, node_list, resource);
    };

    bool NodeAdditions(std::vector<Eigen::MatrixXd>& ff_frames, ikHandler* ik_handler,
                    WM::WM* wm, GeometricFilterHarness* geo_filter, std::vector<node*>& node_map,
                    std::vector<Eigen::VectorXi>& node_list, boost_graph* boost_graph, double resource){
        for (int index=0; index<no_levels; ++index){
            sampler.RandomSample(ff_frames, ik_handler, wm, geo_filter, 
                node_map, node_list, index, isCreated,
                graph_metrics, boost_graph, resource );
        }
        
        return true;
    };
};


#endif























// }

// // Create probabilities for top 2 bottlenecks
// Eigen::VectorXd probs(5);
// probs(4) = 0.2; // Prob for randomly sampling a depth
// probs(3) = 0.1; // Prob for randomly sampling a source
// probs(2) = 0.1; // Prob for randomly sampling a sink
// probs.segment(0,2) = sorted_conn.segment(0,2) / 
//                     (sorted_conn(0)+sorted_conn(1));
// probs.segment(0,2) *= 0.6;

// Eigen::VectorXi depths(5);
// depths.segment(0,2) = sorted_index.segment(0,2);
// // Generate a random level index
// depths(4) = 0 + ( std::rand() % ( no_levels ) );
// // Generate source and sink levels
// depths(2) = 0;
// depths(3) = no_levels-1;

// double prob = (double) std::rand() / RAND_MAX;
// if (prob <= probs(0))
//     index = depths(0);
// if (prob <= probs(0)+probs(1) && prob > probs(0))
//     index = depths(1);
// if (prob <= probs(0)+probs(1)+probs(2) && prob > probs(0)+probs(1))
//     index = depths(2);
// if (prob <= probs(0)+probs(1)+probs(2)+probs(3) && prob > probs(0)+probs(1)+probs(2))
//     index = depths(3);
// if (prob <= 1.0 && prob > probs(0)+probs(1)+probs(2)+probs(3))
//     index = depths(4);

// index = 0 + ( std::rand() % ( no_levels ) );

        // return false;

// // Connection with nodes above and below
// if (index-1 >= 0){
//     MakeConnections( node_list[index-1],node_list[index],edges, 
//                 weights,ik_handler,node_map, boost_graph );
// }

// if (index+1 <= no_levels-1){
//     MakeConnections( node_list[index],node_list[index+1],edges, 
//                 weights,ik_handler,node_map, boost_graph );
// }





// bool use_bias = false;
// if (use_bias){
//     // // Create Bias
//     Eigen::VectorXd node_counts(node_list.size());
//     for (int i=0; i<node_list.size(); ++i)
//         node_counts(i) = node_list[i].size();
//     Eigen::VectorXd score;
//     if (cost_grad.col(0).norm()>1e-8 && cost_grad.col(1).norm()>1e-8){
//         Eigen::VectorXd temp1 = cost_grad.col(0);
//         temp1 = (temp1 / temp1.maxCoeff()) * 0.25;
//         Eigen::VectorXd temp2 = cost_grad.col(1);
//         temp2 = (temp2 / temp2.maxCoeff()) * 0.25;
//         Eigen::VectorXd temp3 = node_counts.array().inverse();
//         temp3 = (temp3 / temp3.maxCoeff()) * 0.5;
//         score = temp1 + temp2 + temp3;
//         Eigen::VectorXd sorted_conn;
//         Eigen::VectorXi sorted_index;
//         sort_vec(score,sorted_conn,sorted_index);
//         int effective_levels = floor(0.3*no_levels);
//         int samples = floor( no_levels / effective_levels );
//         for (int index=0; index<effective_levels; ++index)
//             for (int i=0; i<samples; ++i)
//                 sampler.RandomSample(ff_frames, ik_handler, wm, geo_filter, 
//                     node_map, node_list, sorted_index(index), isCreated,
//                     graph_metrics, boost_graph );
//     }
//     else{
//         for (int index=0; index<no_levels; ++index)
//             sampler.RandomSample(ff_frames, ik_handler, wm, geo_filter, 
//                 node_map, node_list, index, isCreated,
//                 graph_metrics, boost_graph );
//     }
// }
// else{