///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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



int main(int argc, char** argv){
    ros::init(argc,argv,"pct_main");
    ros::NodeHandle main_handler;
    ros::Publisher cvrg_pub = main_handler.advertise<std_msgs::Bool>("cvrg_status",1000);



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


    // std_msgs::Bool msg1;
    // msg1.data = true;
    // ROS_INFO("Publishing Message");
    // ros::Rate loop_rate1(1000);
    // for(int i=0; i<1000; ++i){
    //     cvrg_pub.publish(msg1);
    //     loop_rate1.sleep();
    // }
    // return 0;


    std::cout<< "Obtaining Tool TCP\n";
    // TCP
    // Get transformation from ros parameter server
    std::vector<double> tf; tf.clear();
    if(!ros::param::get("/cvrg_tf_param/ff_T_tool",tf))
        std::cout<< "Unable to Obtain TCP tf\n";
    Eigen::VectorXd tf_eigen(6);
    tf_eigen<< tf[0],tf[1],tf[2],tf[3],tf[4],tf[5];
    Eigen::MatrixXd ff_T_tool = Eigen::MatrixXd::Identity(4,4);
    ff_T_tool.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
    ff_T_tool.block(0,3,3,1) = tf_eigen.segment(0,3);

    std::cout<< "Creating Robot\n";
    // Create the robot
    KDL::Frame tcp_frame = DFMapping::Eigen_to_KDLFrame(ff_T_tool);

    SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, base_frame, tcp_frame, rob_base_link, rob_tip_link);
    Eigen::MatrixXd jt_lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd jt_ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);
    ikHandler ik_handler(&robot);
    ik_handler.init_guess = init_guess.col(0);


    // Create Collision Checker
    WM::WM wm;
    wm.addRobot(robot_obj);
    std::string wp_path;
    if(!ros::param::get("/cvrg_file_paths/mesh_path",wp_path))
        std::cout<< "Unable to Obtain mesh path\n";
    tf.clear();
    if(!ros::param::get("/cvrg_tf_param/world_T_part",tf))
        std::cout<< "Unable to Obtain part tf\n";
    std::string toolstl_path;
    if(!ros::param::get("/cvrg_file_paths/tool_stl_coll_path",toolstl_path))
        std::cout<< "Unable to Obtain tool stl path\n";
    tf_eigen<< tf[0],tf[1],tf[2],tf[3],tf[4],tf[5];
    Eigen::Matrix4d world_T_part = Eigen::Matrix4d::Identity();
    world_T_part.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
    world_T_part.block(0,3,3,1) = tf_eigen.segment(0,3);
    std::vector<Eigen::MatrixXd> zero_fk = robot.get_robot_FK_all_links( Eigen::MatrixXd::Ones(robot.NrOfJoints,1)*0 );
    wm.addTool(toolstl_path);
    wm.prepareSelfCollisionPatch(zero_fk);
    wm.addWorkpiece(wp_path, world_T_part);
    // Eigen::MatrixXd world_T_floor = Eigen::MatrixXd::Identity(4,4); world_T_floor(2,3) += -0.05;
    // wm.addWorkpiece("/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/meshes/floor.stl", world_T_floor);



    // Eigen::MatrixXd path = gen_cvrg_plan();

    std::string file_path = "/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv/backup/";
    Eigen::MatrixXd traj1 = file_rw::file_read_mat(file_path+"0.csv");
    Eigen::MatrixXd traj2 = file_rw::file_read_mat(file_path+"joint_states.csv");

    // Manipulability analysis
    std::cout<< "\nMANIPULABILITY,  Distance to Upper,  Distance to Lower\n";
    double manip1 = 0;
    double manip2 = 0;
    double sum_1 = 0;
    double sum_2 = 0;
    double dist_ub = 0;
    double dist_lb = 0;
    Eigen::MatrixXd ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);
    Eigen::MatrixXd lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd data(traj2.rows(),6);
    for(int i=0; i<traj2.rows(); ++i){
        KDL::Jacobian jac_kdl;
        KDL::JntArray theta;
        Eigen::MatrixXd jac;

        if (i<traj1.rows()){
            theta = DFMapping::Eigen_to_KDLJoints(traj1.row(i).transpose());
            robot.Jac_KDL(theta,jac_kdl);
            jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
            manip1 = sqrt(( jac*jac.transpose() ).determinant());

            data(i,0) = manip1;
            data(i,1) = (ub.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);
            data(i,2) = (lb.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);
            std::cout<< manip1 << "\t\t";
            std::cout<< (ub.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\t\t";
            std::cout<< (lb.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\t\t\t";
        }
        else{
            data(i,0) = 0;
            data(i,1) = 0;
            data(i,2) = 0;
            std::cout<< "NA\t\t" << "NA\t\t" << "NA\t\t" << "\t";
        }

        theta = DFMapping::Eigen_to_KDLJoints(traj2.row(i).transpose());
        robot.Jac_KDL(theta,jac_kdl);
        jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
        manip2 = sqrt(( jac*jac.transpose() ).determinant());

        data(i,3) = manip2;
        data(i,4) = (ub.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);
        data(i,5) = (lb.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);

        std::cout<< manip2 << "\t";
        std::cout<< (ub.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\t\t";
        std::cout<< (lb.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\n";

        if (i<traj1.rows()){
            sum_1 += manip1;
            sum_2 += manip2;
        }
    }
    std::cout<< "Sum of manipulabilities: " << sum_1 << "\t" << sum_2 << "\n";
    file_rw::file_write("/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv/data.csv",data);

    return 0;
}


// Robot jacobian and manipulability is same for different config subsets
// Transform jacobian based on rotation matrices