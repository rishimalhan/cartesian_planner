///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __INCREASEPATHRESOLUTION__HPP__
#define __INCREASEPATHRESOLUTION__HPP__

#include <robot_utilities/ikHandler.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <Eigen/Eigen>

Eigen::MatrixXd IncreasePathResolution(Eigen::MatrixXd traj, ikHandler* ik_handler){
    if (traj.rows()==1)
        return traj;

    int no_samples = 100;
    int counter = 0;
    KDL::Frame fk_kdl;
    KDL::JntArray theta;
    Eigen::MatrixXd fk;
    Eigen::VectorXd target1(12);
    Eigen::VectorXd target2(12);
    Eigen::MatrixXd upd_traj;

    for (int i=0; i<traj.rows()-1; ++i){
        theta = DFMapping::Eigen_to_KDLJoints(traj.row(i).transpose());
        ik_handler->robot->FK_KDL_TCP(theta,fk_kdl);
        fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);

        target1.segment(0,3) = fk.block(0,3,3,1);
        target1.segment(3,3) = fk.block(0,0,3,1);
        target1.segment(6,3) = fk.block(0,1,3,1);
        target1.segment(9,3) = fk.block(0,2,3,1);

        theta = DFMapping::Eigen_to_KDLJoints(traj.row(i+1).transpose());
        ik_handler->robot->FK_KDL_TCP(theta,fk_kdl);
        fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);

        target2.segment(0,3) = fk.block(0,3,3,1);
        target2.segment(3,3) = fk.block(0,0,3,1);
        target2.segment(6,3) = fk.block(0,1,3,1);
        target2.segment(9,3) = fk.block(0,2,3,1);

        ik_handler->init_guess = traj.row(i).transpose();
        Eigen::VectorXd dx = (target2-target1)/no_samples;
        for (int j=0; j<no_samples; ++j){
            ik_handler->solveIK(target1 + j*dx);
            upd_traj.conservativeResize(counter+1,traj.cols());
            upd_traj.row(counter) = ik_handler->closest_sol.transpose();
            counter++;
        }
    }
    return upd_traj;
};

#endif