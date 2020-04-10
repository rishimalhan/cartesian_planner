#include <pct/jacQP_gurobi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <gen_utilities/SerialLink_Manipulator.hpp>
#include <gen_utilities/Data_Format_Mapping.hpp>
#include <Eigen/Eigen>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_qp");
    std::string rob_name = "irb120_3_58";
    // IIWA 7 and 14
    std::string rob_base_link = "base_link";
    std::string rob_tip_link = "tool0";
    // Create a seed for optimizer
    // This will be replaced with more redundant configurations
    Eigen::MatrixXd init_guess(7,1);
    init_guess << 0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463,0;

    std::cout<< "Obtaining Tool TCP\n";
    // TCP
    // Get transformation from ros parameter server
    Eigen::MatrixXd ff_T_tool = Eigen::MatrixXd::Identity(4,4);
    
    std::cout<< "Creating Robot\n";
    // Create the robot
    Eigen::Matrix4d world_T_base = Eigen::Matrix4d::Identity(4,4);

    std::string urdf_path = ros::package::getPath("abb_experimental") + "/abb_irb120_support/urdf/"+rob_name+".urdf";
    KDL::Frame base_frame = KDL::Frame::Identity();
    KDL::Frame tcp_frame = DFMapping::Eigen_to_KDLFrame(ff_T_tool);

    SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, base_frame, tcp_frame, rob_base_link, rob_tip_link);
    Eigen::MatrixXd jt_lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd jt_ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);

    jacQP qp_handler(robot.OptVarDim);
    return 0;
}

