#ifndef __JACDRIVENPCCHECK__HPP__
#define __JACDRIVENPCCHECK__HPP__

#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/transformation_utilities.hpp>



double get_jt_diff(std::vector<Eigen::VectorXd> seg1, std::vector<Eigen::VectorXd> seg2,
                        std::vector<Eigen::MatrixXd> jacobians, ikHandler* ik_handler){
    double step = 1e-3;
    // Drive for Segment-1
    Eigen::VectorXd q = seg1[1];
    Eigen::VectorXd curr_x = seg1[0];
    KDL::Frame fk_kdl;
    KDL::JntArray theta;
    Eigen::VectorXd bxbybz(9);
    int max_itr = 5000;
    int itr = 0;
    while (itr<max_itr){
        // std::cout<< "Current Pose: " << curr_x.transpose() << "\n";
        // std::cout<< "Goal Pose: " << seg1[2].transpose() << "\n\n";
        itr++;
        q += step*(jacobians[0].inverse()*(seg1[2]-curr_x));
        theta = DFMapping::Eigen_to_KDLJoints(q);
        ik_handler->robot->FK_KDL_TCP(theta,fk_kdl);
        Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
        bxbybz.segment(0,3) = fk.block(0,0,3,1);
        bxbybz.segment(3,3) = fk.block(0,1,3,1);
        bxbybz.segment(6,3) = fk.block(0,2,3,1);
        curr_x.segment(0,3) = fk.block(0,3,3,1);
        curr_x.segment(3,3) = rtf::bxbybz2eul(bxbybz.transpose(),"XYZ").row(0).transpose();
        std::cout<< "Error for config-1: " << (seg1[2]-curr_x).norm() << "\n";
        if ( (seg1[2]-curr_x).norm()<1e-2 )
            break;

        // q = seg2[1] + step*(jacobians[1]*(seg2[2]-curr_x));
        // theta = DFMapping::Eigen_to_KDLJoints(q);
        // ik_handler->robot->FK_KDL_TCP(theta,fk_kdl);
        // Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
        // bxbybz.segment(0,3) = fk.block(0,0,3,1);
        // bxbybz.segment(3,3) = fk.block(0,1,3,1);
        // bxbybz.segment(6,3) = fk.block(0,2,3,1);
        // curr_x.segment(0,3) = fk.block(0,3,3,1);
        // curr_x.segment(3,3) = rtf::bxbybz2eul(bxbybz.transpose(),"XYZ").row(0).transpose();
        // std::cout<< "Error for config-1: " << (seg[2]-curr_x).norm() << "\n";
    }
}




// double get_jt_diff(std::vector<Eigen::VectorXd> seg1, std::vector<Eigen::VectorXd> seg2,
//                         std::vector<Eigen::MatrixXd> jacobians, ikHandler* ik_handler){
//     Eigen::VectorXd q1 = seg1[2] + jacobians[0].inverse() * (seg1[4]-seg1[1]);
//     Eigen::VectorXd q2 = seg2[2] + jacobians[1].inverse() * (seg2[4]-seg2[1]);

//     KDL::JntArray theta1;
//     KDL::JntArray theta2;
//     KDL::Jacobian jac_kdl;
//     theta1 = DFMapping::Eigen_to_KDLJoints(q1);
//     ik_handler->robot->Jac_KDL(theta1,jac_kdl);
//     Eigen::MatrixXd jac1 = DFMapping::KDLJacobian_to_Eigen(jac_kdl);

//     theta2 = DFMapping::Eigen_to_KDLJoints(q2);
//     ik_handler->robot->Jac_KDL(theta2,jac_kdl);
//     Eigen::MatrixXd jac2 = DFMapping::KDLJacobian_to_Eigen(jac_kdl);

//     Eigen::VectorXd mid_x_eul = seg1[4];

//     KDL::Frame fk_kdl;
//     Eigen::VectorXd wp_eul(6);
//     Eigen::VectorXd bxbybz(9);

//     // Eliminate error for q1
//     ik_handler->robot->FK_KDL_TCP(theta1,fk_kdl);
//     Eigen::MatrixXd fk1 = DFMapping::KDLFrame_to_Eigen(fk_kdl);
//     bxbybz.segment(0,3) = fk1.block(0,0,3,1);
//     bxbybz.segment(3,3) = fk1.block(0,1,3,1);
//     bxbybz.segment(6,3) = fk1.block(0,2,3,1);
//     wp_eul.segment(0,3) = fk1.block(0,3,3,1);
//     wp_eul.segment(3,3) = rtf::bxbybz2eul(bxbybz.transpose(),"XYZ").row(0).transpose();
//     q1 = q1 + jac1.inverse() * (mid_x_eul - wp_eul);

//     ik_handler->robot->FK_KDL_TCP(theta2,fk_kdl);
//     Eigen::MatrixXd fk2 = DFMapping::KDLFrame_to_Eigen(fk_kdl);
//     bxbybz.segment(0,3) = fk2.block(0,0,3,1);
//     bxbybz.segment(3,3) = fk2.block(0,1,3,1);
//     bxbybz.segment(6,3) = fk2.block(0,2,3,1);
//     wp_eul.segment(0,3) = fk2.block(0,3,3,1);
//     wp_eul.segment(3,3) = rtf::bxbybz2eul(bxbybz.transpose(),"XYZ").row(0).transpose();
//     q2 = q2 + jac2.inverse() * (mid_x_eul - wp_eul);

//     std::cout<< "Config-1 in Jacobian Driving: " << q1.transpose() << "\n";
//     std::cout<< "Config-2 in Jacobian Driving: " << q2.transpose() << "\n";

//     return (q1-q2).norm();
// }



bool JacDrivenPCCheck(std::vector<Eigen::VectorXd> seg, std::vector<Eigen::MatrixXd> jacobians,
                         ikHandler* ik_handler){
    double jt_diff;
    Eigen::VectorXd mid_x_eul = (seg[1] + seg[4])/2;        
    std::vector<Eigen::VectorXd> seg1(3);
    std::vector<Eigen::VectorXd> seg2(3);
    seg1[0] = seg[1];
    seg1[1] = seg[2];
    seg1[2] = mid_x_eul;

    seg2[0] = seg[4];
    seg2[1] = seg[5];
    seg2[2] = mid_x_eul;

    double err = get_jt_diff( seg1,seg2,jacobians,ik_handler );
    std::cout<< "Error: " << err << "\n";
    return true;
}
#endif 