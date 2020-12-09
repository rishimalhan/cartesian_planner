#ifndef __SAVEGRAPHSTATS__
#define __SAVEGRAPHSTATS__

#include <pct/graph_description.hpp>
#include <robot_utilities/file_rw.hpp>

// Find the following for each level
// Valid edge ratio, Valid node ratio, UC, LC, BC, NC

void SaveGraphStats(boost_graph* graph, std::vector<Eigen::VectorXi> node_list,
                    std::vector<Eigen::MatrixXd> ff_frames){
    Eigen::MatrixXd stats = Eigen::MatrixXd::Zero(node_list.size(),5);
    for (int i=0; i<node_list.size(); ++i){
        stats(i,1) = node_list[i].size() / ff_frames[i].rows() / 8; // Node ratio

        for (int k=0; k<node_list[i].size(); ++k){
            // Check for LC
            if (i+1<=node_list.size()-1){
                for (int j=0; j<node_list[i+1].size();++j){
                    if (edge(node_list[i](k), node_list[i+1](j), graph->g).second)
                        stats(i,3)++;
                }
            }

            // Check for UC
            if (i-1>=0){
                for (int j=0; j<node_list[i-1].size();++j){
                    if (edge(node_list[i-1](j), node_list[i](k), graph->g).second)
                        stats(i,2)++;
                }
            }
        }
    }
    ROS_WARN_STREAM("\n\n");
    ROS_WARN_STREAM(stats);
    ROS_WARN_STREAM("\n\n");
};

#endif