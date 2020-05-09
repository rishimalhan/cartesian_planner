///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <robot_utilities/ikHandler.hpp>
#include <robot_utilities/SerialLink_Manipulator.hpp>
#include <Eigen/Eigen>
#include <pct/timer.hpp>
#include <ros/package.h>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/transformation_utilities.hpp>


int main(int argc, char** argv){
    ros::init(argc,argv,"verify_IK");
    ros::NodeHandle main_handler;


    ///////////////// CAUTION ///////////////////////////////////////
    // WHEN CHANGING A 6DOF ROBOT FOR ANALYTICAL IK, MAKE SURE CHANGES ARE MADE
    // IN IK GATEWAY HEADER AND APPROPRIATE CPP IS INCLUDED
    ///////////////// CAUTION ///////////////////////////////////////




    // // ROBOT IIWA
    // std::string rob_name = "iiwa7";
    // // IIWA 7 and 14
    // std::string rob_base_link = "iiwa_link_0";
    // std::string rob_tip_link = "iiwa_link_ee";
    // std::string urdf_path = ros::package::getPath("pct") + "/data/urdf/"+rob_name+".urdf";
    // // Create a seed for optimizer
    // // This will be replaced with more redundant configurations
    // Eigen::MatrixXd init_guess(7,1);
    // init_guess << 0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463,0;
    // KDL::Frame base_frame = KDL::Frame::Identity();



    // ROBOT IRB2600
    Eigen::MatrixXd init_guess(6,1);
    init_guess << 0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463;
    std::string rob_base_link = "base_link";
    std::string rob_tip_link = "tool0";
    std::string urdf_path = ros::package::getPath("robot_utilities") + "/urdf/abb_irb2600/irb2600_12_165.urdf";
    std::string robot_obj = ros::package::getPath("robot_utilities") + "/rob_objs/abb_irb2600/";
    KDL::Frame base_frame = KDL::Frame::Identity();




    // // ROBOT UR10e
    // Eigen::MatrixXd init_guess(6,1);
    // init_guess << 0,-90,0,0,0,0;
    // std::string rob_base_link = "base_link";
    // std::string rob_tip_link = "tool0";
    // std::string urdf_path = ros::package::getPath("robot_utilities") + "/urdf/ur_10e/ur10e.urdf";
    // std::string robot_obj = ros::package::getPath("robot_utilities") + "/rob_objs/ur_10e/";
    // Eigen::VectorXd w_T_b_eul(3);
    // w_T_b_eul << M_PI,0,0;
    // Eigen::MatrixXd w_T_b = Eigen::MatrixXd::Identity(4,4);
    // w_T_b.block(0,0,3,3) = rtf::eul2rot(w_T_b_eul.transpose(),"ZYX");
    // KDL::Frame base_frame = DFMapping::Eigen_to_KDLFrame(w_T_b);
    // // KDL::Frame base_frame = KDL::Frame::Identity();
    bool urPatch = false;


    // std_msgs::Bool msg1;
    // msg1.data = true;
    // ROS_INFO("Publishing Message");
    // ros::Rate loop_rate1(1000);
    // for(int i=0; i<1000; ++i){
    //     cvrg_pub.publish(msg1);
    //     loop_rate1.sleep();
    // }
    // return 0;


    KDL::Frame tcp_frame = KDL::Frame::Identity();
    std::cout<< "Creating Robot\n";
    // Create the robot
    SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, base_frame, tcp_frame, rob_base_link, rob_tip_link);
    Eigen::MatrixXd jt_lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd jt_ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);
    ikHandler ik_handler(&robot);
    ik_handler.init_guess = init_guess.col(0);
    if (urPatch)
        ik_handler.enable_URikPatch();

    int no_samples = 10000;
    int no_fails = 0;

    // timer timer;
    // timer.start();
    // // Randomly Sample and Check IK
    // for (int i=0; i<no_samples; ++i){
    //     KDL::JntArray theta;
    //     robot.getRandJointArray(theta);
    //     KDL::Frame fk_kdl;
    //     robot.FK_KDL_Flange(theta, fk_kdl);
    //     Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
    //     Eigen::VectorXd target(12);
    //     target.segment(0,3) = fk.block(0,3,3,1);
    //     target.segment(3,3) = fk.block(0,0,3,1);
    //     target.segment(6,3) = fk.block(0,1,3,1);
    //     target.segment(9,3) = fk.block(0,2,3,1);
    //     if(!ik_handler.solveIK(target)){
    //         no_fails++;
    //         // std::cout<< "Bug in IK Solver\n";
    //         // break;
    //     }
    // }
    // std::cout<< "Time for one IK: " << timer.elapsed()/no_samples << "\n";
    // std::cout<< "IK success rate: " << (1 - no_fails/no_samples)*100 << "\n";


    // Check numerical IK
    ik_handler.useNumIK = true;
    ik_handler.init_guess << 0,0,0,0,0,0;
    timer timer;
    timer.start();
    // Randomly Sample and Check IK
    for (int i=0; i<no_samples; ++i){
        KDL::JntArray theta;
        robot.getRandJointArray(theta);
        KDL::Frame fk_kdl;
        robot.FK_KDL_Flange(theta, fk_kdl);
        Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
        Eigen::VectorXd target(12);
        target.segment(0,3) = fk.block(0,3,3,1);
        target.segment(3,3) = fk.block(0,0,3,1);
        target.segment(6,3) = fk.block(0,1,3,1);
        target.segment(9,3) = fk.block(0,2,3,1);
        if(!ik_handler.solveIK(target)){
            no_fails++;
            // std::cout<< "Bug in IK Solver\n";
            // break;
        }
    }
    std::cout<< "Time for one IK: " << timer.elapsed()/no_samples << "\n";
    std::cout<< "IK success rate: " << (1 - no_fails/no_samples)*100 << "\n";    
    return 0;
}