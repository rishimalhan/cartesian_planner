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
#include <pct/path_consistency.hpp>




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

    // // Manipulability analysis
    // std::cout<< "\nMANIPULABILITY,  Distance to Upper,  Distance to Lower\n";
    // double manip1 = 0;
    // double manip2 = 0;
    // double sum_1 = 0;
    // double sum_2 = 0;
    // double dist_ub = 0;
    // double dist_lb = 0;
    // Eigen::MatrixXd ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);
    // Eigen::MatrixXd lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    // Eigen::MatrixXd data(traj2.rows(),6);
    KDL::Jacobian jac_kdl;
    KDL::JntArray theta;
    Eigen::MatrixXd jac;


    // for(int i=0; i<traj2.rows(); ++i){
    //     if (i<traj1.rows()){
    //         theta = DFMapping::Eigen_to_KDLJoints(traj1.row(i).transpose());
    //         robot.Jac_KDL(theta,jac_kdl);
    //         jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    //         manip1 = sqrt(( jac*jac.transpose() ).determinant());

    //         data(i,0) = manip1;
    //         data(i,1) = (ub.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);
    //         data(i,2) = (lb.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);
    //         std::cout<< manip1 << "\t\t";
    //         std::cout<< (ub.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\t\t";
    //         std::cout<< (lb.array() - traj1.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\t\t\t";
    //     }
    //     else{
    //         data(i,0) = 0;
    //         data(i,1) = 0;
    //         data(i,2) = 0;
    //         std::cout<< "NA\t\t" << "NA\t\t" << "NA\t\t" << "\t";
    //     }

    //     theta = DFMapping::Eigen_to_KDLJoints(traj2.row(i).transpose());
    //     robot.Jac_KDL(theta,jac_kdl);
    //     jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    //     manip2 = sqrt(( jac*jac.transpose() ).determinant());

    //     data(i,3) = manip2;
    //     data(i,4) = (ub.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);
    //     data(i,5) = (lb.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI);

    //     std::cout<< manip2 << "\t";
    //     std::cout<< (ub.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\t\t";
    //     std::cout<< (lb.array() - traj2.row(i).transpose().array()).abs().minCoeff()*(180/M_PI) << "\n";

    //     if (i<traj1.rows()){
    //         sum_1 += manip1;
    //         sum_2 += manip2;
    //     }
    // }
    // std::cout<< "Sum of manipulabilities: " << sum_1 << "\t" << sum_2 << "\n";
    // file_rw::file_write("/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv/data.csv",data);






    // Analysis of dx - Jdq
    Eigen::VectorXd target1(12);
    Eigen::VectorXd target2(12);

    // target1<< 0.33857,-0.189353,0.337679,0.999886,-0.00159247,
    // -0.0150355,-0.00163258,-0.999995,-0.0026558,-0.0150312,0.00268005,-0.999883;

    // target2 = target1;
    // target2(0) += 0.1;
    // target2(1) += 0.1;
    // target2(2) += 0.1;
    

    // target1 << 0.97553 ,  0.0160197 ,   0.236795, 0.000609452,    0.765329,   -0.643639 ,   0.945616,   -0.209807,   -0.248578 ,  -0.325284,   -0.608484 ,  -0.723835;
    // target2 << 0.975506,  -0.0139803,    0.258972, 0.000670697,    0.842239,   -0.539105,     0.95532,   -0.159884,   -0.248597,   -0.295572 ,  -0.514851,   -0.804715;


    // // Test Case-1
    // target1 << 0.975546,0.0360197,0.21907,-0.00058108,-0.729701,0.683766,-0.949522,0.2149,0.22853,-0.3137,-0.649118,-0.692992;
    // target2 << 0.975538,0.0260197,0.228244,-0.00060237,-0.756436,0.654068,-0.954525,0.195432,0.225139,-0.298129,-0.624189,-0.722155;

    // Test Case-2
    target1 << 0.975268,-0.31398,0.3204,-0.000781082,-0.980856,-0.194732,-0.940122,-0.0656515,0.334455,-0.340836,0.183333,-0.922074;
    target2 << 0.97526,-0.32398,0.318245,-0.000776739,-0.975402,-0.220432,-0.955684,-0.06417,0.287317,-0.294395,0.210887,-0.932126;

    // int i=0; 
    // int no_samples = 100;
    // // ik_handler.useNumIK = true;
    // // ik_handler.init_guess << 17.7609,  20.9944,  37.9902,  -33.721,  70.2479, -41.8573;
    // // ik_handler.init_guess *= (M_PI/180);
    // Eigen::VectorXd dx = (target2-target1)/no_samples;
    // while (i<no_samples){
    //     Eigen::VectorXd target = target1+i*dx;
    //     if (ik_handler.solveIK(target)){
    //         theta = DFMapping::Eigen_to_KDLJoints(ik_handler.solution.col(0));
    //         KDL::Frame fk_kdl;
    //         robot.FK_KDL_TCP(theta,fk_kdl);
    //         Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
    //         // std::cout<< fk.block(0,3,3,1).transpose() << " " << fk.block(0,0,3,1).transpose()
    //         // << " " << fk.block(0,1,3,1).transpose()
    //         // << " " << fk.block(0,2,3,1).transpose() << "\n";
    //         robot.Jac_KDL(theta,jac_kdl);
    //         jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    //         std::cout<< "Config: "<< i << ": " << ik_handler.solution.col(0).transpose()*(180/M_PI) << " Manip: " <<
    //         sqrt(( jac*jac.transpose() ).determinant()) << "\n";
    //     }
    //     i++;
    // }

    // return 0;

    std::vector<Eigen::VectorXd> seg(4);
    seg[0] = target1;
    seg[2] = target2;

    if (ik_handler.solveIK(target1))
        seg[1] = ik_handler.solution.col(0);
    else
        return 0;
    if (ik_handler.solveIK(target2))
        seg[3] = ik_handler.solution.col(1);
    else
        return 0;


    std::cout<< "Config-1: " << seg[1].transpose()*(180/M_PI) << "\n";
    std::cout<< "Config-2: " << seg[3].transpose()*(180/M_PI) << "\n";

    std::cout<< "Consistency Status: "<< 
    path_consistency(seg, &ik_handler, get_dist(seg, &ik_handler)) << "\n";
    return 0;









    // std::vector<Eigen::VectorXd> segment(4);

    // ik_handler.solveIK(target1);
    // Eigen::VectorXd config1 = ik_handler.solution.col(0);
    // ik_handler.solveIK(target2);    
    // Eigen::VectorXd config2 = ik_handler.solution.col(0);

    // Eigen::VectorXd dq = (config2-config1)/100;
    // for (int i=0; i<100; ++i){
    //     theta = DFMapping::Eigen_to_KDLJoints(config1+i*dq);
    //     KDL::Frame fk_kdl;
    //     robot.FK_KDL_TCP(theta,fk_kdl);
    //     Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);   
    //     // std::cout<< fk.block(0,3,3,1).transpose() << "\n";
    // }

    // // Analyzing Path Consistency




    // ik_handler.solveIK(target1);
    // config1 = ik_handler.solution.col(0);

    // for (int j=1; j<100; j++){
    //     double i = 100;
    //     target2 = target1;
    //     target2(0) += (double)i/1000;
    //     target2(1) += (double)i/1000;
    //     target2(2) += (double)i/1000;

    //     // Eigen::MatrixXd eul_angle = rtf::bxbybz2eul(target2.segment(3,9).transpose(),"XYZ");
    //     // eul_angle.row(0) += Eigen::MatrixXd::Ones(1,3)*(double)i/100;
    //     // target2.segment(3,9) = rtf::eul2bxbybz(eul_angle,"XYZ").row(0).transpose();

    //     Eigen::VectorXd x1(6);
    //     x1.segment(0,3) = target1.segment(0,3); 
    //     x1.segment(3,3) = rtf::bxbybz2eul(target1.segment(3,9).transpose(),"XYZ").row(0).transpose();

    //     Eigen::VectorXd x2(6);
    //     x2.segment(0,3) = target2.segment(0,3); 
    //     x2.segment(3,3) = rtf::bxbybz2eul(target2.segment(3,9).transpose(),"XYZ").row(0).transpose();

    //     if(!ik_handler.solveIK(target2))
    //         break;
    //     Eigen::MatrixXd sol2 = ik_handler.solution;
    //     Eigen::VectorXd config2;

    //     if (j<5){
    //         config2 = sol2.col(0).transpose();
    //     }
    //     else{
    //         config2 = sol2.col(1).transpose();
    //     }

    //     // std::cout<< "Config-1: " << config1.transpose()*(180/M_PI) << "\n";
    //     // std::cout<< "Config-2: " << config2.transpose()*(180/M_PI) << "\n";
    //     // std::cout<< "Joint Difference Norm: " << (config2-config1).norm()*(180/M_PI) << "\n"; 

    //     theta = DFMapping::Eigen_to_KDLJoints(config1);
    //     robot.Jac_KDL(theta,jac_kdl);
    //     jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);

    //     // std::cout<< "Distance: " << j*5 << " mm. Area: " << ( (x2-x1)-jac*(config2-config1) ).norm() << "\n";
    //     target1 = target2;
    //     config1 = config2;
    // }



    return 0;
}


// Robot jacobian and manipulability is same for different config subsets
// Transform jacobian based on rotation matrices