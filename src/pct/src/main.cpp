#include <iostream>
#include <pct/gen_cvrg_plan.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <pct/genTolWp.hpp>
#include <gen_utilities/file_rw.hpp>
#include <gen_utilities/transformation_utilities.hpp>
#include <pct/jacQP.hpp>
#include <gen_utilities/SerialLink_Manipulator.hpp>
#include <Eigen/Eigen>
#include <gen_utilities/Data_Format_Mapping.hpp>
#include <boost/date_time.hpp>


int main(int argc, char** argv){
    // int b = 2;
    // long int sum = 1;
    // for (int i=1; i<=10; ++i)
    //     sum += pow(b,i);
    // std::cout<< sum << "\n";
    // return 0;

    ros::init(argc,argv,"main");
    ros::NodeHandle main_handler;
    ros::Publisher cvrg_pub = main_handler.advertise<std_msgs::Bool>("cvrg_plan",1000);
    Eigen::MatrixXd path = gen_cvrg_plan();
    std_msgs::Bool msg;
    msg.data = true;
    ROS_INFO("Publishing Message");
    ros::Rate loop_rate(1000);
    for(int i=0; i<10; ++i){
        cvrg_pub.publish(msg);
        loop_rate.sleep();
    }

    // Obtain all the waypoints with tolerances applied at different depths
    double resolution = 10*M_PI / 180;
    double angle = 90*M_PI / 180;
    std::cout<< "Generating Search Samples....\n";
    Eigen::MatrixXd tolerances = Eigen::MatrixXd::Ones(path.rows(),1)*angle;
    std::vector<Eigen::MatrixXd> wpTol =  gen_wp_with_tolerance(tolerances,resolution, path );


    // Convert all bxbybz to eul angles


    // Call the Search
    Eigen::Matrix4d world_T_base = Eigen::Matrix4d::Identity(4,4);

    std::string urdf_path = ros::package::getPath("pct") + "/data/urdf/iiwa7.urdf";
    KDL::Frame base_frame = KDL::Frame::Identity();
    KDL::Frame tcp_frame = KDL::Frame::Identity();
    // IIWA 7 and 14
    std::string rob_base_link = "iiwa_link_0";
    std::string rob_tip_link = "iiwa_link_ee";

    SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, base_frame, tcp_frame, rob_base_link, rob_tip_link);
    Eigen::MatrixXd jt_lb = DFMapping::KDLJoints_to_Eigen(robot.Joints_ll);
    Eigen::MatrixXd jt_ub = DFMapping::KDLJoints_to_Eigen(robot.Joints_ul);



    // Create a dummy example
    Eigen::MatrixXd init_guess(robot.NrOfJoints,1);

    init_guess << 0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463,0;

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

    std::cout<< "Solving IK......\n";

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

    double elapsed;
    boost::posix_time::ptime start_time;
    boost::posix_time::time_duration time_diff;
    start_time = boost::posix_time::microsec_clock::local_time();


    numIK ik_handler(&robot);
    Eigen::VectorXd jt_config;
    for(int i=0; i<1000; ++i)
        jt_config = ik_handler.solveIK(init_guess.col(0),target);    

    time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = time_diff.total_nanoseconds() / 1e9;  
    std::cout<< "compute time: " << elapsed << std::endl;


    std::cout<< "#################################################\n";

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

    return 0;
}