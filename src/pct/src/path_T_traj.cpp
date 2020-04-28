///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 
// E:222600800 Breakdown
// N:651705 E:125212950 Breakdown
// N:434470 E:55650200 Works nut 7 GB memory
// N:347576 E:35616128 Safe limit 4 GB memory

#include <iostream>
#include <pct/gen_cvrg_plan.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <pct/genTolWp.hpp>
#include <robot_utilities/file_rw.hpp>
#include <robot_utilities/transformation_utilities.hpp>
#include <robot_utilities/ikHandler.hpp>
#include <robot_utilities/SerialLink_Manipulator.hpp>
#include <Eigen/Eigen>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/world_manager.hpp>
#include <pct/ss_searches.hpp>
#include <pct/node_description.hpp>
#include <pct/gen_nodes.hpp>
#include <pct/build_graph.hpp>
#include <pct/graph_description.hpp>
#include <pct/graph_searches.hpp>
#include <pct/timer.hpp>


// Parameters to play with:
// Parameters to check while things are not working out
// Which robot is enabled. Check urdf, tcp, base frames and links
// Check for which header is included in ik gateway
// idedge return true or false for general cases in build graph
// Is path being generated or loaded
// Tool transforms and planner.yaml config file
// Remember to switch useNumIK back to false if done using it


int main(int argc, char** argv){
    ros::init(argc,argv,"path_T_traj");
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



    // // ROBOT IRB2600
    // Eigen::MatrixXd init_guess(6,1);
    // init_guess << 0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463;
    // std::string rob_base_link = "base_link";
    // std::string rob_tip_link = "tool0";
    // std::string urdf_path = ros::package::getPath("robot_utilities") + "/urdf/abb_irb2600/irb2600_12_165.urdf";
    // std::string robot_obj = ros::package::getPath("robot_utilities") + "/rob_objs/abb_irb2600/";
    // KDL::Frame base_frame = KDL::Frame::Identity();




    // ROBOT UR10e
    Eigen::MatrixXd init_guess(6,1);
    init_guess << 0.05416107177734375,-1.337559537296631, 1.9834006468402308, -0.6330326360515137, 1.6855134963989258, -0.004259411488668263;
    std::string rob_base_link = "base_link";
    std::string rob_tip_link = "tool0";
    std::string urdf_path = ros::package::getPath("robot_utilities") + "/urdf/ur_10e/ur10e.urdf";
    std::string robot_obj = ros::package::getPath("robot_utilities") + "/rob_objs/ur_10e/";
    Eigen::VectorXd w_T_b_eul(3);
    w_T_b_eul << M_PI,0,0;
    Eigen::MatrixXd w_T_b = Eigen::MatrixXd::Identity(4,4);
    w_T_b.block(0,0,3,3) = rtf::eul2rot(w_T_b_eul.transpose(),"ZYX");
    KDL::Frame base_frame = DFMapping::Eigen_to_KDLFrame(w_T_b);
    // KDL::Frame base_frame = KDL::Frame::Identity();
    bool urPatch = true;


    std::cout<< "Creating Robot\n";
    // Create the robot
    KDL::Frame tcp_frame = DFMapping::Eigen_to_KDLFrame(Eigen::MatrixXd::Identity(4,4));

    SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, base_frame, tcp_frame, rob_base_link, rob_tip_link);
    Eigen::MatrixXd jt_lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd jt_ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);
    ikHandler ik_handler(&robot);
    ik_handler.init_guess = init_guess.col(0);
    if (urPatch)
        ik_handler.enable_URikPatch();

    std::string file_name;    
    std::cin >> file_name;

    std::string path_file = "/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv/" + file_name;
    Eigen::MatrixXd path = file_rw::file_read_mat(path_file); // Pre computed path file


    // Get trajectory path
    std::string traj_path = "/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv/joint_states.csv";
    Eigen::MatrixXd trajectory;
    
    timer timer;
    timer.start();
    for (int i=0; i<path.rows(); ++i){
        if (ik_handler.solveIK(path.row(i).transpose())){
            // std::cout<< "IK feasible for waypoint: "<< i+1 << "\n";
            trajectory.conservativeResize(i+1,ik_handler.OptVarDim);
            trajectory.row(i) = ik_handler.closest_sol;
        }
        else{
            std::cout<< "IK not found\n";
            break;
        }
    }
    std::cout<< "Number of Points: " << path.rows() << ". Compute time: " << timer.elapsed() << "\n";
    file_rw::file_write(traj_path,trajectory);

    return 0;
}