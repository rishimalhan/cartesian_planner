///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
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



int main(int argc, char** argv){
    ros::init(argc,argv,"pct_main");
    ros::NodeHandle main_handler;
    ros::Publisher cvrg_pub = main_handler.advertise<std_msgs::Bool>("cvrg_status",1000);



    double resolution;
    if(!ros::param::get("/sampling_res",resolution)){
        std::cout<< "Unable to Obtain Sampling Resolution\n";
        return 0;
    }
    resolution *= (M_PI / 180);
    double angle = 360*M_PI / 180;


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
    init_guess << 0,-90,0,0,0,0;
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
    // wm.addWorkpiece(wp_path, world_T_part);
    Eigen::MatrixXd world_T_floor = Eigen::MatrixXd::Identity(4,4); world_T_floor(2,3) += -0.05;
    wm.addWorkpiece("/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/meshes/floor.stl", world_T_floor);

    std::string path_file;
    ros::param::get("/cvrg_file_paths/path_file",path_file);

    // Generate Path
    // Eigen::MatrixXd path = gen_cvrg_plan();
    Eigen::MatrixXd path = load_plan(path_file); // Pre computed path file


    // removeRow(path,path.rows()-1);
    // removeRow(path,path.rows()-2);
    // file_rw::file_write(cvrg_path,path);



    Eigen::MatrixXd f1(6,1);
    f1 << 0,0,0,0,0,0;
    KDL::JntArray f1_kdl = DFMapping::Eigen_to_KDLJoints(f1);
    KDL::Frame fra;
    robot.FK_KDL_Flange(f1_kdl,fra);
    std::cout<< w_T_b * DFMapping::KDLFrame_to_Eigen(fra) << "\n";
    return 0;



    // Obtain all the waypoints with tolerances applied at different depths
    // Note these points will be the transformed points in space
    std::cout<< "Generating Search Samples....\n";
    Eigen::MatrixXd tolerances = Eigen::MatrixXd::Ones(path.rows(),1)*angle;
    std::vector<Eigen::MatrixXd> wpTol =  gen_wp_with_tolerance(tolerances,resolution, path );
    std::cout<< "Search Samples Generated....\n";


    // Get trajectory path
    std::string traj_path;
    ros::param::get("/cvrg_file_paths/joint_states",traj_path);
    Eigen::MatrixXd trajectory;
    Eigen::MatrixXd success_flags = Eigen::MatrixXd::Ones(path.rows(),1)*0;
    std::string success_flag_path;
    ros::param::get("/cvrg_file_paths/success_flags",success_flag_path);




    // Generate tcp_list
    double no_tcps;
    if(!ros::param::get("/no_tcps",no_tcps)){
        std::cout<< "Unable to Obtain Number of TCPs\n";
        return 0;
    }
    std::cout<< "Number of TCPs: " << no_tcps << "\n";
    std::vector<Eigen::MatrixXd> tcp_list(no_tcps); // For now only 1 tcp
    for (int i=0; i<tcp_list.size(); ++i)
        tcp_list[i] = ff_T_tool;

    


// ////////////////////////////////////////////////////////////////////
//     // test collision

//     std::vector<double> config; config.clear();
//     Eigen::MatrixXd jt_pt(6,1);
//     std_msgs::Bool msg1;
//     msg1.data = true;
//     ROS_INFO("Publishing Message");
//     ros::Rate loop_rate1(10);
//     while(ros::ok()){
//         ros::param::get("/rob_config",config);
//         jt_pt << config[0], config[1],config[2],config[3], config[4],config[5];
//         jt_pt *= (M_PI/180);
//         std::vector<Eigen::MatrixXd> fk_kdl = robot.get_robot_FK_all_links(jt_pt);
//         std::cout<< "Collision Status: " << wm.inCollision( fk_kdl ) << "\n";
//         trajectory.resize(1,6);
//         trajectory.row(0) = jt_pt.col(0).transpose();
//         file_rw::file_write(traj_path,trajectory);
//         cvrg_pub.publish(msg1);
//         loop_rate1.sleep();
//     }
// ////////////////////////////////////////////////////////////////////








    // double elapsed;
    // boost::posix_time::ptime start_time;
    // boost::posix_time::time_duration time_diff;
    // start_time = boost::posix_time::microsec_clock::local_time();

    timer timer;
    timer.start();


    // // Using Sequential IK
    // std::cout<< "Solving IK......\n";
    // Eigen::VectorXd jt_config;
    // if (ik_handler.solveIK(path.row(0))){ // IK for first row
    //     jt_config = ik_handler.solution.col(0); // First solution out of many
    //     ik_handler.init_guess = jt_config;
    //     std::cout<< "The first configuration in degrees is: \n"
    //     << ik_handler.init_guess.transpose()*180/M_PI << "\n\n";
    //     double acc_error = 0;
    //     for (int i=0; i<path.rows(); ++i){
    //         if (ik_handler.solveIK(path.row(i)))
    //             jt_config = ik_handler.closest_sol;
    //         else
    //             break;
    //         trajectory.conservativeResize(i+1,robot.NrOfJoints);
    //         trajectory.row(i) = jt_config.transpose();
    //         ik_handler.init_guess = jt_config;
    //         success_flags(i,0) = ik_handler.status;
    //         acc_error += ik_handler.f_val;
    //     }
    //     std::cout<< "Trajectory\n" << trajectory << "\n";
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
    //         if (ik_handler.solveIK(path.row(i)))
    //             Eigen::VectorXd jt_config = ik_handler.solution;
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

    

    // // Tree Search
    // if(ik_handler.solveIK(path.row(0))){
    //     ik_handler.init_guess.col(0) = ik_handler.solution.col(0);
    //     std::cout<< "The first configuration in degrees is: \n"
    //     << ik_handler.init_guess.col(0).transpose()*180/M_PI << "\n\n";
    // }

    // double acc_error = 0;
    // ss_searches search_handler;
    // Eigen::MatrixXd ret_val;
    // if(!search_handler.djk_v1(&ik_handler, &wm, wpTol,ret_val)){
        // std::cout<< "Search Failed. No solution found\n";
        // trajectory.resize(1,ik_handler.OptVarDim);
        // trajectory.row(0) << ik_handler.init_guess.col(0).transpose();
    // }
    // else{
    //     trajectory = ret_val.block(0,0,ret_val.rows(),robot.NrOfJoints);
    //     success_flags = ret_val.block(0,robot.NrOfJoints,ret_val.rows(),1);
    // }
    // std::cout<< trajectory << "\n";



    // Graph Search
    std::vector<node*> node_map;
    std::vector<Eigen::VectorXi> node_list;
    success_flags = Eigen::MatrixXd::Ones(wpTol.size(),1)*0;
    if(!gen_nodes(&ik_handler, &wm, wpTol, tcp_list, node_map, node_list, success_flags)){
        std::cout<< "Nodes could not be generated. No solution found\n";
        trajectory.resize(1,ik_handler.OptVarDim);
        trajectory.row(0) << ik_handler.init_guess.col(0).transpose();
    }
    else{
        std::cout<< "Nodes generation compute time: " << timer.elapsed() << std::endl;
        boost_graph graph;
        std::vector<bool> root_connectivity;
        if(!build_graph(node_map,node_list,&graph,root_connectivity)){
            std::cout<< "Edges could not be created. No solution found\n";
            trajectory.resize(1,ik_handler.OptVarDim);
            trajectory.row(0) << ik_handler.init_guess.col(0).transpose();
        }
        else{
            std::cout<< "Graph generation compute time: " << timer.elapsed() << std::endl;
            Eigen::VectorXi strt_nodes = node_list[0];
            double lowest_cost = std::numeric_limits<double>::infinity();
            // Run Search For Multiple Start Configs
            for (int i=0; i<strt_nodes.size(); ++i){
                if (!root_connectivity[i])
                    continue;
                int root_node = strt_nodes(i);
                Eigen::VectorXd root_config = node_map[root_node]->jt_config;
                std::cout<< "Robot Config in Degrees: " << root_config.transpose()*(180/M_PI) << "\n";
                // Change the start vertex in graph
                vertex_descriptor s = vertex(root_node, graph.g); graph.s = s;
                Eigen::VectorXi id_path;
                if(graph_searches::djikstra(&graph,id_path)){ // Get the shortest path to a leaf node in terms of node ids
                    // Generate Trajectory
                    Eigen::MatrixXd curr_traj(wpTol.size(), robot.NrOfJoints);
                    for(int i=0; i<id_path.size(); ++i)
                        curr_traj.row(i) = node_map[id_path(i)]->jt_config.transpose();
                    // Evaluate trajectory cost
                    double path_cost = 0;
                    for (int i=0; i<curr_traj.rows()-1;++i){
                        Eigen::ArrayXd jt_diff = (curr_traj.row(i+1) - curr_traj.row(i)).transpose();
                        path_cost += jt_diff.abs().maxCoeff();
                    }
                    if (path_cost<lowest_cost){
                        std::cout<< "Found Path With Lower Cost. Current Path Cost: " << path_cost << "\n\n";
                        lowest_cost = path_cost;
                        trajectory = curr_traj;
                    }
                    success_flags = Eigen::MatrixXd::Ones(wpTol.size(),1);
                }
                else
                    std::cout<< "Discontinuity detected for this configuration\n";
            }

            // std::cout<< "Trajectory: \n" << trajectory << "\n";
        }
    }



    

    // Evaluate trajectory cost
    double max_change = -std::numeric_limits<double>::infinity();
    double path_cost = 0;
    for (int i=0; i<trajectory.rows()-1;++i){
        Eigen::ArrayXd jt_diff = (trajectory.row(i+1) - trajectory.row(i)).transpose();
        path_cost += jt_diff.abs().maxCoeff();
        if (jt_diff.abs().maxCoeff()>max_change)
            max_change = jt_diff.abs().maxCoeff();
    }
    std::cout<< "Maximum Joint Angle Change in Trajectory: " << max_change*(180/M_PI) << "\n";
    std::cout<< "Number of waypoints: " << wpTol.size() << "\n";
    std::cout<< "Number of TCPs: " << no_tcps << "\n";
    std::cout<< "Trajectory Cost: " << path_cost << "\n";
    std::cout<< "Total compute time: " << timer.elapsed() << std::endl;

    file_rw::file_write(traj_path,trajectory);
    file_rw::file_write(success_flag_path,success_flags);
    std::cout<< "#################################################\n";



    std_msgs::Bool msg;
    msg.data = true;
    ROS_INFO("Publishing Message");
    ros::Rate loop_rate(1000);
    for(int i=0; i<1000; ++i){
        cvrg_pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}



/*
// CODE NOT REQUIRED FOR NOW



// ik_handler.enable_URikPatch();
    // // Check Ariyan's points
    // std::cout<< "\n\n\n";
    // Eigen::MatrixXd points(4,6);
    // points<< -27.96, -309.59, 1487.29, 1.978, 0.103, -5.386,
    //         -404.92, 386.56, 190.81, 2.559, 0.648, 0.228,
    //         -120.09, 358.45, 972.77, 0.026, -0.014, -0.162,
    //         -211.58, 29.11, 1091.52, 1.041, -0.180, -5.663;
    // points.block(0,0,points.rows(),3) /= 1000;
    // Eigen::MatrixXd jt_pts(4,6);
    // jt_pts<< -61.54, -100.26, 5.04, 195.66, -92.28, 0.82,
    //         -61.46, -79.36, 128.56, 250.54, -92.27, 0.82,
    //         -99.51, -100.25, 92.53, 96.18, -89.31, -0.01,
    //         -61.46, -117.34, 92.36, 119.93, -92.27, 0.82;
    // jt_pts *= (M_PI/180) ;




    // // IK Check
    // for (int i=0; i<points.rows();++i){
    //     Eigen::VectorXd target(12);
    //     target.segment(3,9) = rtf::eul2bxbybz(points.block(i,3,1,3),"ZYX").row(0).transpose();
    //     target.segment(0,3) = points.block(i,0,1,3).transpose();
    //     std::cout<< "Target: \n" << target.transpose() << "\n";
    //     ik_handler.init_guess = jt_pts.row(i).transpose();
    //     if (ik_handler.solveIK(target)){
    //         std::cout<< "Closest Solution for point: " << i+1 << "\n";
    //         std::cout<< ik_handler.closest_sol.transpose()*(180/M_PI) << "\n\n";
    //         std::cout<< "KDL FK: \n";
    //         KDL::Frame frame;
    //         KDL::JntArray fuck = DFMapping::Eigen_to_KDLJoints(ik_handler.closest_sol);
    //         robot.FK_KDL_Flange(fuck,frame);
    //         std::cout<< w_T_b*DFMapping::KDLFrame_to_Eigen(frame) << "\n\n";
    //     }
    //     else
    //         std::cout<< "Fucked!\n";
    // }
    // std::cout<< "\n\n\n";


    // std::cout<< "\n\n\n";
    // // FK check
    // for (int i=0; i<jt_pts.rows();++i){
    //     std::cout<< "KDL FK: \n";
    //     KDL::Frame frame;
    //     KDL::JntArray fuck = DFMapping::Eigen_to_KDLJoints(jt_pts.row(i).transpose());
    //     robot.FK_KDL_Flange(fuck,frame);
    //     std::cout<< w_T_b*DFMapping::KDLFrame_to_Eigen(frame) << "\n\n";
    // }
    // std::cout<< "\n\n\n";

    // trajectory.resize(1,6);
    // trajectory.row(0) << 0, -90, 0, -90, 0, 0;


    // trajectory.row(0) *= (M_PI/180);
    // Eigen::MatrixXd jt_config = trajectory.row(0).transpose();
    // std::cout<< "KDL FK: \n";
    // KDL::Frame frame;
    // KDL::JntArray fuck = DFMapping::Eigen_to_KDLJoints(jt_config);
    // robot.FK_KDL_Flange(fuck,frame);
    // std::cout<< w_T_b*DFMapping::KDLFrame_to_Eigen(frame) << "\n\n";




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
    // ikHandler ik_handler(&robot);
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

// // Debug configs manipulability. Delete later
    // std::string t_path;
    // if(!ros::param::get("/cvrg_file_paths/joint_states",t_path))
    //     ROS_INFO("Unable to obtain robot trajectory");
    // Eigen::MatrixXd traj = file_rw::file_read_mat(t_path);

    // std::cout<< "Joints: \n" << traj << "\n";

    // for (int i=0; i<traj.rows();++i){
    //     KDL::Jacobian jac;
    //     KDL::JntArray jt = DFMapping::Eigen_to_KDLJoints(traj.row(i).transpose());
    //     robot.Jac_KDL(jt,jac);
    //     Eigen::MatrixXd jacobian = DFMapping::KDLJacobian_to_Eigen(jac);
    //     std::cout<< sqrt((jacobian*jacobian.transpose()).determinant()) << "\n";
    // }
    // return 0;

    // KDL::JntArray t_config(6);
    // t_config(0) = 0.00775273;
    // t_config(1) = 0.112036;
    // t_config(2) = 0.650733;
    // t_config(3) = 0.028248;
    // t_config(4) = 1.05047;
    // t_config(5) = -0.00839303;

    // t_config(0) = 0;
    // t_config(1) = 0;
    // t_config(2) = 0;
    // t_config(3) = 0;
    // t_config(4) = 0;
    // t_config(5) = 0;

    // Eigen::VectorXd target_bxbybz(12);
    // KDL::Frame kdl_fk;
    // robot.FK_KDL_TCP(t_config,kdl_fk);
    // Eigen::MatrixXd target_fk = DFMapping::KDLFrame_to_Eigen(kdl_fk);
    // std::cout<< "Target: \n" << target_fk << "\n";
    // target_bxbybz.segment(0,3) = target_fk.block(0,3,3,1);
    // target_bxbybz.segment(3,3) = target_fk.block(0,0,3,1);
    // target_bxbybz.segment(6,3) = target_fk.block(0,1,3,1);
    // target_bxbybz.segment(9,3) = target_fk.block(0,2,3,1);

    // if (ik_handler.solveIK(target_bxbybz)){
    //     std::cout<< "Success\n";
    //     Eigen::MatrixXd solution = ik_handler.solution;
    //     std::cout<< solution << "\n";
    // }
    // return 0;
*/