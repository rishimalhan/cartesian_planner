#ifndef __SHORTESTDISTANCECHECK__HPP__
#define __SHORTESTDISTANCECHECK__HPP__

#include <Eigen/Geometry> 
#include <robot_utilities/transformation_utilities.hpp>
#include <cmath>

double GetWSDist(Eigen::VectorXd c1, Eigen::VectorXd c2,
                 ikHandler* ik_handler){
    int no_samples = 10;
    Eigen::VectorXd dq = (c2 - c1)/no_samples;
    double pos_dist = 0;
    double ori_dist = 0;
    Eigen::VectorXd prev_point(7);
    for (int i=0; i<=no_samples; ++i){
        KDL::JntArray theta = DFMapping::Eigen_to_KDLJoints(c1 + i*dq);
        KDL::Frame fk_kdl;
        ik_handler->robot->FK_KDL_TCP(theta,fk_kdl); // Tool tip
        Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
        Eigen::VectorXd wp_quat(7);
        wp_quat.segment(0,3) = fk.block(0,3,3,1);
        wp_quat.segment(3,4) = rtf::rot2qt(fk.block(0,0,3,3)).row(0).transpose();

        // std::cout<< fk.block(0,3,3,1).transpose() << " " << fk.block(0,0,3,1).transpose() << " " << 
        // fk.block(0,1,3,1).transpose() << " " <<  fk.block(0,2,3,1).transpose() << "\n";
        
        if (i==0){
            prev_point = wp_quat;
            continue;
        }
        pos_dist += (wp_quat.segment(0,3)-prev_point.segment(0,3)).norm();
        ori_dist += std::fabs(2 * acos( std::fabs(wp_quat.segment(3,4).dot(prev_point.segment(3,4))) ));
        prev_point = wp_quat;
    }
    return (pos_dist+ori_dist);
}


bool ShortestDistanceCheck(std::vector<Eigen::VectorXd> seg, ikHandler* ik_handler){
    // For all possible configurations, generate more segments
    if (ik_handler->solveIK(seg[2])){
        std::vector<double> dist; dist.clear();
        // std::cout<< "Solutions: " << ik_handler->solution.transpose() * (180/M_PI) << "\n";
        for (int i=0; i<ik_handler->solution.cols(); ++i){
            // std::cout<< "Solution: " << ik_handler->solution.col(i).transpose()*(180/M_PI) << "\n";
            double distance = GetWSDist(seg[1],ik_handler->solution.col(i),ik_handler);
            dist.push_back( distance );
            // std::cout<< "Positional Distance: " << distances[0] << ". Orientational Distance: " 
            // << distances[1] << "\n";
        }
        if ( (ik_handler->solution.col(std::min_element(dist.begin(),dist.end()) - dist.begin())-
                seg[3]).norm() > 1e-2 )
            return false;
        else
            return true;
    }
    else{
        // std::cout<< "BUGGGGG!!!! IK not feasible\n";
    }
    return false;
}

#endif 