#ifndef __GEN_WDG_HPP__
#define __GEN_WDG_HPP__

#include <iostream>
#include <Eigen/Eigen>
#include <robot_utilities/ikHandler.hpp>


bool isChild(Eigen::VectorXd wp, Eigen::VectorXd child, double tol){
    bool isChild = false;
    // Eigen::MatrixXd wR1 = rtf::eul2rot(rtf::bxbybz2eul(wp.segment(3,9).transpose()),"ZYX");
    // Eigen::MatrixXd wR2 = rtf::eul2rot(rtf::bxbybz2eul(child.segment(3,9).transpose()),"ZYX");
    // Eigen::MatrixXd _1R2 = wR1.transpose() * wR2;
    // Eigen::MatrixXd eul_angles = rtf::rot2eul(_1R2);
    // std::cout<< "Relative Euler: " << eul_angles*180/M_PI << "\n";

    double angle_x = acos( wp.segment(3,3).transpose()*child.segment(3,3) );
    if (angle_x <= tol )
        isChild = true;
    return isChild;
};

static bool gen_WDG(ikHandler* ik_handler, const std::vector<Eigen::MatrixXd> wpTol, int config_idx,
                       std::vector<std::vector<int>>& node_ids, int& node_cnt, std::vector<std::vector<Eigen::VectorXi>>& WDG,
                       std::vector<Eigen::MatrixXd>& wpTol_filtered){ // output is wpTol_filtered and WDG
    bool status;
    double max_deviation = 30*M_PI / 180; // All nodes within x degrees of parent are children

    wpTol_filtered.clear();

    for (int ctr=0; ctr<wpTol.size();++ctr){
        Eigen::MatrixXd waypoints = wpTol[ctr];
        Eigen::MatrixXd new_wps;
        int k = 0;
        status = false;
        if (ctr==0){
            new_wps.conservativeResize(k+1,13); // Last column is the original index
            new_wps.row(k) << waypoints.row(config_idx),config_idx;
            wpTol_filtered.push_back(new_wps);
            continue;
        }
        for (int i=0; i<waypoints.rows();++i){
            if (ik_handler->solveIK(waypoints.row(i))) {
                new_wps.conservativeResize(k+1,12); // Last column is the original index
                new_wps.row(k) << waypoints.row(i);
                k++;
                status = true;
            }
        }
        if (!status){
            std::cout<< "Some waypoints can't be reached!! WDG failed to generate\n";
            break;
        }
        else{
            std::cout<< "Waypoint: " << ctr + 1 << " can be reached.\n";
            wpTol_filtered.push_back(new_wps);
        }
    }
    if (status){
        std::cout<< "All waypoints reachable!! Depth of filtered wpTol: " << wpTol_filtered.size() << "\n";

        // Initialize Data Structures
        // Node id for i,j elements in wpTol can be accessed by using i and j
        std::cout<< "Initializing Node IDs.......\n";
        int ctr = 0;
        node_ids.clear();
        
        for (int i=0; i<wpTol_filtered.size();++i){
            std::vector<int> temp1; temp1.clear();
            for (int j=0; j<wpTol_filtered[i].rows();++j){
                temp1.push_back(ctr);
                ctr++;
            }
            node_ids.push_back(temp1);
        }
        node_cnt = ctr;
        std::cout<< "Number of Nodes Created: " << node_cnt << "\n";


        // Filtering children to be considered for evaluation at each parent node
        // For every depth and index (a node), it stores the indices of children.
        for (int i=0; i<wpTol_filtered.size()-1; ++i){
            Eigen::MatrixXd wps = wpTol_filtered[i];
            Eigen::MatrixXd nxt_wps = wpTol_filtered[i+1];
            std::vector<Eigen::VectorXi> temp; temp.clear();
            for (int j=0; j<wps.rows(); ++j){
                int cnt = 0;
                Eigen::VectorXi children;
                for (int k=0; k<nxt_wps.rows(); ++k){
                    if(isChild(wps.row(j),nxt_wps.row(k),max_deviation)){
                        children.conservativeResize(cnt+1);
                        children(cnt) = k;
                        cnt++;
                    }
                }
                temp.push_back(children);
            }
            WDG.push_back(temp);
        }
        std::cout<< "WDG Generated. Depth of WDG: " << WDG.size() << "\n";
    }
    return status;
};

#endif