#include <iostream>
#include <pct/gen_cvrg_plan.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <pct/genTolWp.hpp>
#include <gen_utilities/file_rw.hpp>
#include <gen_utilities/transformation_utilities.hpp>
#include <pct/numIK.hpp>
#include <gen_utilities/SerialLink_Manipulator.hpp>
#include <Eigen/Eigen>
#include <gen_utilities/Data_Format_Mapping.hpp>
#include <boost/date_time.hpp>
#include <gen_utilities/world_manager.hpp>
#include <pct/ss_searches.hpp>



int main(int argc, char** argv){
    double resolution = 36*M_PI / 180;
    double angle = 360*M_PI / 180;


    std::string rob_name = "iiwa7";
    // IIWA 7 and 14
    std::string rob_base_link = "iiwa_link_0";
    std::string rob_tip_link = "iiwa_link_ee";
    // Create a seed for optimizer
    // This will be replaced with more redundant configurations
    Eigen::MatrixXd init_guess(7,1);
    init_guess << 0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463,0;



    ros::init(argc,argv,"pct_main");
    ros::NodeHandle main_handler;
    ros::Publisher cvrg_pub = main_handler.advertise<std_msgs::Bool>("cvrg_status",1000);

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
    Eigen::Matrix4d world_T_base = Eigen::Matrix4d::Identity(4,4);

    std::string urdf_path = ros::package::getPath("pct") + "/data/urdf/"+rob_name+".urdf";
    KDL::Frame base_frame = KDL::Frame::Identity();
    KDL::Frame tcp_frame = DFMapping::Eigen_to_KDLFrame(ff_T_tool);

    SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, base_frame, tcp_frame, rob_base_link, rob_tip_link);
    Eigen::MatrixXd jt_lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd jt_ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);
    numIK ik_handler(&robot);


    // Create Collision Checker







    Eigen::MatrixXd path = gen_cvrg_plan();
    removeRow(path,path.rows()-1);
    removeRow(path,path.rows()-2);
    std::string cvrg_path;
    ros::param::get("/cvrg_file_paths/cvrg_path",cvrg_path);
    file_rw::file_write(cvrg_path,path);



    // Obtain all the waypoints with tolerances applied at different depths
    // Note these points will be the transformed points in space
    std::cout<< "Generating Search Samples....\n";
    Eigen::MatrixXd tolerances = Eigen::MatrixXd::Ones(path.rows(),1)*angle;
    std::vector<Eigen::MatrixXd> wpTol =  gen_wp_with_tolerance(tolerances,resolution, path );

    // Get trajectory path
    std::string traj_path;
    ros::param::get("/cvrg_file_paths/joint_states",traj_path);
    Eigen::MatrixXd trajectory;
    Eigen::MatrixXd success_flags = Eigen::MatrixXd::Ones(path.rows(),1)*0;
    std::string success_flag_path;
    ros::param::get("/cvrg_file_paths/success_flags",success_flag_path);




    // Using Waypoint Tolerance and Search
    ik_handler.init_guess = init_guess.col(0);




    double elapsed;
    boost::posix_time::ptime start_time;
    boost::posix_time::time_duration time_diff;
    start_time = boost::posix_time::microsec_clock::local_time();



    // // Using Sequential IK
    // std::cout<< "Solving IK......\n";
    // ik_handler.init_guess = init_guess.col(0);
    // double acc_error = 0;
    // for (int i=0; i<path.rows(); ++i){
    //     Eigen::VectorXd jt_config = ik_handler.solveIK(path.row(i));
    //     trajectory.conservativeResize(i+1,robot.NrOfJoints);
    //     trajectory.row(i) = jt_config.transpose();
    //     ik_handler.init_guess = jt_config;
    //     success_flags(i,0) = ik_handler.status;
    //     acc_error += ik_handler.f_val;
    // }




    // // Using Random Initial Seeds
    // // Using Sequential IK
    // std::cout<< "Solving IK......\n";
    // ik_handler.init_guess = init_guess.col(0);
    // double min_err = 100000;
    // double acc_error;
    // for (int ctr=0; ctr<30; ++ctr){
    //     acc_error = 0;
    //     KDL::JntArray randJoints(robot.NrOfJoints);
    //     robot.getRandJointArray(randJoints);
    //     init_guess << DFMapping::KDLJoints_to_Eigen(randJoints);
    //     std::cout<< "Seed: " << init_guess.transpose() << "\n";
    //     for (int i=0; i<path.rows(); ++i){
    //         Eigen::VectorXd jt_config = ik_handler.solveIK(path.row(i));
    //         trajectory.conservativeResize(i+1,robot.NrOfJoints);
    //         trajectory.row(i) = jt_config.transpose();
    //         ik_handler.init_guess = jt_config;
    //         success_flags(i,0) = ik_handler.status;
    //         acc_error += ik_handler.f_val;
    //     }
    //     if (acc_error < min_err)
    //         min_err = acc_error;
    // }
    // acc_error = min_err;    

    

    double acc_error = 0;
    ss_searches search_handler;
    Eigen::MatrixXd ret_val;
    if(!search_handler.djk_v1(&ik_handler, wpTol,ret_val)){
        std::cout<< "Search Failed. No solution found\n";
        trajectory.resize(1,ik_handler.OptVarDim);
        trajectory.row(0) << ik_handler.init_guess;
    }
    else{
        trajectory = ret_val.block(0,0,ret_val.rows(),robot.NrOfJoints);
        success_flags = ret_val.block(0,robot.NrOfJoints,ret_val.rows(),1);
    }



    std::cout<< "Average Error Per WayPoint: " << acc_error/trajectory.rows() << "\n";
    time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = time_diff.total_nanoseconds() / 1e9;  
    std::cout<< "compute time: " << elapsed << std::endl;

    file_rw::file_write(traj_path,trajectory);
    file_rw::file_write(success_flag_path,success_flags);
    std::cout<< "#################################################\n";



    std_msgs::Bool msg;
    msg.data = true;
    ROS_INFO("Publishing Message");
    ros::Rate loop_rate(1000);
    for(int i=0; i<100; ++i){
        cvrg_pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}



/*
// CODE NOT REQUIRED FOR NOW
// init_guess << 0,0,0,0,0,0,0;
KDL::JntArray theta = DFMapping::Eigen_to_KDLJoints(init_guess); 
KDL::Frame base_T_tcp;
robot.FK_KDL_TCP(theta,base_T_tcp); // Frame for the configuration
Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(base_T_tcp);
KDL::Jacobian jac_kdl;
robot.Jac_KDL(theta,jac_kdl);
Eigen::MatrixXd jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);

// Eigen::MatrixXd x1(6,1);
// x1.block(0,0,3,1) = fk.block(0,3,3,1); 
// x1.block(3,0,3,1) = rtf::rot2eul(fk.block(0,0,3,3),"XYZ").row(0).transpose();

// Eigen::MatrixXd x2(6,1);
// x2 = x1;
// x2(0,0) += 0.005;

// std::cout<< x1 << "\n\n";
// std::cout<< x2 << "\n";




// std::cout<< fk << "\n\n";
// // std::cout<< x1 << "\n\n";
// // std::cout<< x2 << "\n\n";
// std::cout<< jac << "\n\n";
// std::cout<< init_guess << "\n\n";
// std::cout<< jt_lb << "\n\n";
// std::cout<< jt_ub << "\n\n";

// jacQP qp_handler(init_guess.rows());
// Eigen::VectorXd jt_config = init_guess + qp_handler.solveQP( init_guess, jac, (x2-x1),jt_lb,jt_ub );

Eigen::VectorXd target(12);
target.segment(0,3) = fk.block(0,3,3,1);
target.segment(3,3) = fk.block(0,0,3,1); 
target.segment(6,3) = fk.block(0,1,3,1);
target.segment(9,3) =  fk.block(0,2,3,1);
target(0) += 0.005;


// x1.block(0,0,3,1) *= 1000;
// x1.block(3,0,3,1) *= (180/M_PI);
// std::cout<< "Initial Pose: " << x1.transpose() << "\n";

// x2.block(0,0,3,1) *= 1000;
// x2.block(3,0,3,1) *= (180/M_PI);
// std::cout<< "Target: " << x2.transpose() << "\n";

// theta = DFMapping::Eigen_to_KDLJoints(jt_config); 
// robot.FK_KDL_TCP(theta,base_T_tcp);
// fk = DFMapping::KDLFrame_to_Eigen(base_T_tcp);
// Eigen::MatrixXd x3(6,1);
// x3.block(0,0,3,1) = fk.block(0,3,3,1); 
// x3.block(3,0,3,1) = rtf::rot2eul(fk.block(0,0,3,3),"XYZ").row(0).transpose();    

// x3.block(0,0,3,1) *= 1000;
// x3.block(3,0,3,1) *= (180/M_PI);
// std::cout<< "Reached: " << x3.transpose() << "\n";



std::cout<< "Target: " << target.transpose() << "\n";

theta = DFMapping::Eigen_to_KDLJoints(jt_config); 
robot.FK_KDL_TCP(theta,base_T_tcp);
fk = DFMapping::KDLFrame_to_Eigen(base_T_tcp);

target.segment(0,3) = fk.block(0,3,3,1);
target.segment(3,3) = fk.block(0,0,3,1); 
target.segment(6,3) = fk.block(0,1,3,1);
target.segment(9,3) =  fk.block(0,2,3,1);
std::cout<< "Reached: " << target.transpose() << "\n";

std::cout<< "J1: " << init_guess*(180/M_PI) << "\n";
std::cout<< "J2: " << jt_config*(180/M_PI) << "\n";
KDL::JntArray solution = DFMapping::Eigen_to_KDLJoints(jt_config);

// // Check effect of waypoint tolerance
    // // Eigen::MatrixXd wp_path = wpTol[200];
    // Eigen::MatrixXd wp_path = path;
    // success_flags.resize(wp_path.rows(),1);
    // std::cout<< "Solving IK......\n";
    // numIK ik_handler(&robot);
    // ik_handler.init_guess = init_guess.col(0);
    // double acc_error = 0;
    // for (int i=0; i<wp_path.rows(); ++i){
    //     Eigen::VectorXd jt_config = ik_handler.solveIK(wp_path.row(i));
    //     trajectory.conservativeResize(i+1,robot.NrOfJoints); // Last column shows if it's reachable
    //     trajectory.row(i) = jt_config.transpose();
    //     ik_handler.init_guess = jt_config;
    //     success_flags(i,0) = ik_handler.status;
    //     acc_error += ik_handler.f_val;
    // }
*/