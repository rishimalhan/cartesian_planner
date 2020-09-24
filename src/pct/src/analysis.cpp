///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// #define DEBUG_PATH_CONSISTENCY
// #define DEBUG_CVG


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
// #include <pct/path_consistency.hpp>
// #include <pct/path_consistency_convergence.hpp>



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
    // By default it takes the first tf as a nominal TCP
    std::string tool_name;
    if(!ros::param::get("/tool_name",tool_name))
        ROS_WARN("Unable to Obtain Tool name");
    std::vector<double> tf; tf.clear();
    tool_name = "/" + tool_name + "/ff_T_tool";
    if(!ros::param::get(tool_name,tf)){
        ROS_WARN("Unable to Obtain TCP tf");
        return 1;
    }

    
    int no_tcps = tf.size()/6;
    std::vector<Eigen::MatrixXd> tcp_list(no_tcps);
    Eigen::VectorXd tf_eigen(6);
    for (int i=0; i<no_tcps; ++i){
        tf_eigen<< tf[i*6],tf[i*6+1],tf[i*6+2],tf[i*6+3],tf[i*6+4],tf[i*6+5];
        Eigen::MatrixXd ff_T_tool = Eigen::MatrixXd::Identity(4,4);
        ff_T_tool.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
        ff_T_tool.block(0,3,3,1) = tf_eigen.segment(0,3);
        tcp_list[i] = ff_T_tool;
    }

    std::cout<< "Creating Robot\n";
    // Create the robot
    KDL::Frame tcp_frame = DFMapping::Eigen_to_KDLFrame(tcp_list[0]);

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
    double manip1 = 0;
    double manip2 = 0;
    double sum_1 = 0;
    double sum_2 = 0;
    double dist_ub = 0;
    double dist_lb = 0;
    Eigen::MatrixXd ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);
    Eigen::MatrixXd lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd data(traj2.rows(),6);
    KDL::Jacobian jac_kdl;
    KDL::JntArray theta;
    Eigen::MatrixXd jac;


    // std::cout<< "\nMANIPULABILITY,  Distance to Upper,  Distance to Lower\n";
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


    // return 0;



    // Analysis of dx - Jdq
    Eigen::VectorXd target1(12);
    Eigen::VectorXd target2(12);

    // target1<< 0.33857,-0.189353,0.337679,0.999886,-0.00159247,
    // -0.0150355,-0.00163258,-0.999995,-0.0026558,-0.0150312,0.00268005,-0.999883;

    // target2 = target1;
    // target2(0) += 0.1;
    // target2(1) += 0.1;
    // target2(2) += 0.1;
    


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
            // KDL::Frame fk_kdl;
            // robot.FK_KDL_TCP(theta,fk_kdl);
            // Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
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


    // Case when configs c1 and c2 are inconsistent but function gives consistent
    // as the threshold is too high at 0.9 and reduction of 15% is obtained
    Eigen::VectorXd c1(6);
    Eigen::VectorXd c2(6);
    c1<< -26.8803 ,-8.23767,  64.1847, -148.633, -21.8895,  35.7246;
    c2<< -22.5127, -9.55133,  67.3078,  27.8599,  11.8884, -139.383;

    c1<< 6.83634, -8.38785,   64.222,        0,  34.1659, -171.295;
    c2<< 5.56762, -9.43446,  65.0595,        0,   34.375,  69.2048;

    c1<< 6.83634, -8.38785,   64.222,        0,  34.1659,   68.2048;
    c2<< 5.56762, -9.43446,  65.0595,        0,   34.375,  69.2048;

    // Condition which is violating PC theorm
    // c1<< -38.0349,  3.19692,  53.3185,      180, -33.4846, -145.285;
    // c2<< -37.1694,  5.41314,  50.8849,        0,  33.7019, -77.1237;


    // c1<< -146.702, -39.1332,  61.4869,  304.566,  33.6173, -186.856;
    // c2<< -149.644, -41.4227,   60.955,  304.136,  31.5812, -185.963;

    // c1 *= (3.14/180);
    // c2 *= (3.14/180);



    // // I think following example is violation of config having pos and ori both mini but is still PC
    // target1 << -0.065971,   1.32866,  0.633328, -0.540029, -0.820469, -0.187615,
    //  0.0540645,  -0.25627,  0.965092, -0.839908,  0.511035,  0.182752;
    // target2 << -0.127249,   1.24218,  0.591808,  -0.44703, -0.852819, -0.269932,
    //   0.147047, -0.367717,  0.918238,  -0.88235,  0.370787,  0.289785;
    // c1 << -102.32,  -53.943, -139.149, -74.9189,  74.3156, -27.9487;
    // c2 << -100.425, -53.5368, -146.277, -69.8999,  84.5695, -38.9972;

    // c1 *= (3.14/180);
    // c2 *= (3.14/180);

    
    // std::vector<Eigen::VectorXd> segment(4);
    // segment[0] = target1;
    // segment[1] = c1;
    // segment[2] = target2;
    // segment[3] = c2;

    // std::cout<< "Validity: " << ShortestDistanceCheck(segment, &ik_handler) << "\n";

    // return 0;



    // // Shouldn't be PC but it is
    // target1 <<  -1.19093, -0.249334, -0.176893, -0.436431,  0.589835,  0.679428,
    //  -0.782622,  0.123677, -0.610087 , -0.44388, -0.797997,  0.407641;
    // target2 << -0.519246,  0.175798,   2.14568,  0.587851,  -0.75536,  -0.28959,
    //   0.755462,  0.384562,  0.530461, -0.289324, -0.530606,  0.796711;
    // c1 <<  177.243,  89.2407, -3.66856, -61.9959, -112.034, -133.797;
    // c2 <<  137.664,  30.9181, -109.385, -62.9435, -41.2411, -135.765;

    // c1 *= (3.14/180);
    // c2 *= (3.14/180);

    
    // std::vector<Eigen::VectorXd> segment(4);
    // segment[0] = target1;
    // segment[1] = c1;
    // segment[2] = target2;
    // segment[3] = c2;
    // std::cout<< "Validity: " << ShortestDistanceCheck(segment, &ik_handler) << "\n";

    // return 0;



    // Violating PC rule for weighted sum
    c1 << -13.6384, -1.67241,  33.3332,        0,  58.3393,  -97.998;
    c2 << -17.0218,  6.18898,  25.1781,        0,  58.6329,  35.7281;
    c1 *= (3.14/180);
    c2 *= (3.14/180);

    KDL::Frame fk_kdl;
    theta = DFMapping::Eigen_to_KDLJoints(c1);
    robot.FK_KDL_TCP(theta,fk_kdl);
    Eigen::MatrixXd fk1 = DFMapping::KDLFrame_to_Eigen(fk_kdl);

    theta = DFMapping::Eigen_to_KDLJoints(c2);
    robot.FK_KDL_TCP(theta,fk_kdl);
    Eigen::MatrixXd fk2 = DFMapping::KDLFrame_to_Eigen(fk_kdl);


    target1.segment(0,3) = fk1.block(0,3,3,1);
    target1.segment(3,3) = fk1.block(0,0,3,1);
    target1.segment(6,3) = fk1.block(0,1,3,1);
    target1.segment(9,3) = fk1.block(0,2,3,1);

    target2.segment(0,3) = fk2.block(0,3,3,1);
    target2.segment(3,3) = fk2.block(0,0,3,1);
    target2.segment(6,3) = fk2.block(0,1,3,1);
    target2.segment(9,3) = fk2.block(0,2,3,1);


    Eigen::VectorXd dx = (target2 - target1) / 100;
    ik_handler.init_guess = c1;
    for (int i=0; i<100; ++i){
        ik_handler.solveIK(target1+i*dx);
        std::cout<< ik_handler.closest_sol.transpose() << "\n";
    }
    return 0;

    std::vector<Eigen::VectorXd> segment(4);
    segment[0] = target1;
    segment[1] = c1;
    segment[2] = target2;
    segment[3] = c2;
    // std::cout<< "Validity: " << ShortestDistanceCheck(segment, &ik_handler) << "\n";

    return 0;


    // ik_handler.jt_ll << -3.14, -2.705, -2.705, -3.14, -2.094, -3.14;
    // ik_handler.jt_ul << 3.14, 1.658, 1.309, 3.14, 2.094, 3.14;

    // int no_samples = 50000;

    // timer timer;
    // timer.start();
    // double tot_time = 0;
    // for (int i=0; i<no_samples; ++i){
    //     KDL::JntArray sample;
    //     robot.getRandJointArray(sample);
    //     Eigen::VectorXd c1 = DFMapping::KDLJoints_to_Eigen(sample).col(0);
        
    //     if (!ik_handler.IsWithinLimits(c1))
    //         continue;

    //     theta = DFMapping::Eigen_to_KDLJoints(c1);
    //     robot.Jac_KDL(theta,jac_kdl);
    //     jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    //     double manip = (jac*jac.transpose()).determinant();

    //     if (manip < 0.2)
    //         continue;

    //     Eigen::VectorXd perturb = Eigen::VectorXd::Random(6);
    //     perturb = perturb * 0.3;
        
    //     c2 = c1 + perturb;

    //     // if ( (c2-c1).norm() < 1.57 )
    //     //     continue;

    //     bool flag = false;
    //     Eigen::VectorXd dq = (c2 - c1)/no_samples;
    //     for (int j=0; j<=no_samples; ++j){
    //         if (!ik_handler.IsWithinLimits(c1 + j*dq)){
    //             flag = true;
    //             break;
    //         }
    //     }
    //     if (flag)
    //         continue;

        // KDL::Frame fk_kdl;
        // theta = DFMapping::Eigen_to_KDLJoints(c1);
        // robot.FK_KDL_TCP(theta,fk_kdl);
        // Eigen::MatrixXd fk1 = DFMapping::KDLFrame_to_Eigen(fk_kdl);

        // theta = DFMapping::Eigen_to_KDLJoints(c2);
        // robot.FK_KDL_TCP(theta,fk_kdl);
        // Eigen::MatrixXd fk2 = DFMapping::KDLFrame_to_Eigen(fk_kdl);


        // target1.segment(0,3) = fk1.block(0,3,3,1);
        // target1.segment(3,3) = fk1.block(0,0,3,1);
        // target1.segment(6,3) = fk1.block(0,1,3,1);
        // target1.segment(9,3) = fk1.block(0,2,3,1);

        // target2.segment(0,3) = fk2.block(0,3,3,1);
        // target2.segment(3,3) = fk2.block(0,0,3,1);
        // target2.segment(6,3) = fk2.block(0,1,3,1);
        // target2.segment(9,3) = fk2.block(0,2,3,1);

    //     Eigen::VectorXd eul1 = rtf::bxbybz2eul( target1.segment(3,9).transpose(),"XYZ" ).row(0).transpose();
    //     Eigen::VectorXd eul2 = rtf::bxbybz2eul( target2.segment(3,9).transpose(),"XYZ" ).row(0).transpose();

    //     // std::cout<< "Pos Diff: " << (target2.segment(0,3) - target1.segment(0,3)).norm()
    //     //              << "Angle Diff: " << (eul2 - eul1).norm() * (180/M_PI) << "\n";

    //     // ik_handler.solveIK(target1);
    //     // std::cout<< ik_handler.solution.transpose() * (180/M_PI) << "\n\n";

    //     // ik_handler.solveIK(target2);
    //     // std::cout<< ik_handler.solution.transpose() * (180/M_PI) << "\n\n";

    //     std::vector<Eigen::VectorXd> seg1(4);
    //     seg1[0] = target1;
    //     seg1[1] = c1;
    //     seg1[2] = target2;
    //     seg1[3] = c2;


    //     timer.reset();
    //     if (!ShortestDistanceCheck(seg1, &ik_handler)){
    //         std::cout<< "Invalid for: " << i << "\n";
    //         std::cout<< "Target1: " << target1.transpose() << "\n";
    //         std::cout<< "Target2: " << target2.transpose() << "\n";
    //         std::cout<< "C1: " << c1.transpose() * (180/M_PI) << "\n";
    //         std::cout<< "C2: " << c2.transpose() * (180/M_PI) << "\n";
    //     }
    //     tot_time += timer.elapsed();
    //     continue;
    //     std::cout<< "Consistency Status: "<< 
    //     // path_consistency(seg1, &ik_handler, get_dist(seg1, &ik_handler)) << "\n";
    //     ShortestDistanceCheck(seg1, &ik_handler) << "\n";
    // }
    // std::cout<< "Check time: " << tot_time / no_samples << "\n" ;
    // return 0;





    std::vector<Eigen::VectorXd> seg(6);
    std::vector<Eigen::MatrixXd> jacobians(2);
    target1<< 0.975268,-0.31398,0.3204,-0.000781082,-0.980856,-0.194732,-0.940122,-0.0656515,0.334455,-0.340836,0.183333,-0.922074;
    target2<< 0.975244,-0.34398,0.313075,-0.000763927,-0.959313,-0.282343,-0.933548,-0.100522,0.344069,-0.358452,0.263844,-0.895488;


    seg[0] = target1;
    seg[3] = target2;

    if (ik_handler.solveIK(target1))
        seg[2] = ik_handler.solution.col(0);
    else
        return 0;
    if (ik_handler.solveIK(target2))
        seg[5] = ik_handler.solution.col(0);
    else
        return 0;

    Eigen::VectorXd wp_eul(6);
    // bxbybz to euler waypoint
    wp_eul.resize(6);
    wp_eul.segment(0,3) = target1.segment(0,3); 
    wp_eul.segment(3,3) = rtf::bxbybz2eul(target1.segment(3,9).transpose(),"XYZ").row(0).transpose();
    seg[1] = wp_eul;

    // bxbybz to euler waypoint
    wp_eul.resize(6);
    wp_eul.segment(0,3) = target2.segment(0,3); 
    wp_eul.segment(3,3) = rtf::bxbybz2eul(target2.segment(3,9).transpose(),"XYZ").row(0).transpose();
    seg[4] = wp_eul;

    std::cout<< "Config-1: " << seg[2].transpose()*(180/M_PI) << "\n";
    std::cout<< "Config-2: " << seg[5].transpose()*(180/M_PI) << "\n";

    theta = DFMapping::Eigen_to_KDLJoints(seg[2]);
    robot.Jac_KDL(theta,jac_kdl);
    jacobians[0] = DFMapping::KDLJacobian_to_Eigen(jac_kdl);

    theta = DFMapping::Eigen_to_KDLJoints(seg[5]);
    robot.Jac_KDL(theta,jac_kdl);
    jacobians[1] = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    

    // std::cout<< "Consistency Status: "<< 
    // path_consistency(seg, &ik_handler, get_dist(seg, &ik_handler)) << "\n";
    // path_consistency_cvg(seg, &ik_handler, get_dist(seg, &ik_handler)) << "\n";
    // JacDrivenPCCheck(seg, jacobians, &ik_handler);

    return 0;






// Case violating shortest distance consistency check
// start:  6.83287  -8.3836   64.1894   0  34.1486 -171.208
// end:    5.5648   -9.42968  65.0265   0  34.3576  69.1697

// Solution:  6.20451  163.096   131.851  0     155.008   128.986
// Solution:  6.20451  163.096   131.851  180  -155.008  -51.0136
// Solution:  6.20451  -8.90682  64.6111  0     34.2502   128.986
// Solution:  6.20451  -8.90682  64.6111  180  -34.2502   -51.0135
// Solution: -173.795  -14.6082  156.843  180   52.1887   128.986
// Solution: -173.795  -14.6082  156.843  0    -52.1887   -51.0135
// Solution: -173.795  -149.952  39.6194  180   159.621   128.986
// Solution: -173.795  -149.952  39.6194  0    -159.621   -51.0136

// q1:  6.20451 -8.90682  64.6111      180 -34.2502 -51.0135
// q2:  6.20451 -8.90682  64.6111      180 -34.2502 -51.0135





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