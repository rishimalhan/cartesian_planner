///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 
// E:222600800 Breakdown
// N:651705 E:125212950 Breakdown
// N:434470 E:55650200 Works but 7 GB memory
// N:347576 E:35616128 Safe limit 4 GB memory


// #define DEBUG_MODE_MAIN
// #define SEQ_IK
// #define DEBUG_PATH_CONSISTENCY
#define GRAPH_SEARCH
// #define SEARCH_ASSERT
// #define BASELINE
#define PCT_PLANNER


// #define fileWrite

#include <iostream>
#include <pct/gen_cvrg_plan.hpp>
#include <ros/ros.h>
#include <ros/package.h>
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
#ifdef BASELINE
#include <pct/gen_nodes.hpp>
#include <pct/build_graph.hpp>
#endif
#include <pct/graph_description.hpp>
#include <pct/graph_searches.hpp>
#include <pct/timer.hpp>
#include <pct/IncreasePathResolution.hpp>
#include <pct/geometric_filter.h>
#include <gen_utilities/utilities.hpp>
#include <unordered_set>
#ifdef PCT_PLANNER
#include <pct/PCTplanner.hpp>
#endif
#include <random>
#include <pct/SaveGraphStats.hpp>

// Test Cases
// roslaunch pct bootstrap.launch part:=fender tool:=cam_sander_0 viz:=sim
// roslaunch pct bootstrap.launch part:=step_slab tool:=ferro_sander viz:=sim
// roslaunch pct bootstrap.launch part:=boeing tool:=cam_sander_90 viz:=sim
// roslaunch pct bootstrap.launch part:=gear_int tool:=ati viz:=sim
// roslaunch pct bootstrap.launch part:=bath_tub tool:=ferro_sander viz:=sim


// Parameters to play with:
// Parameters to check while things are not working out
// Which robot is enabled. Check urdf, tcp, base frames and links
// Check for which header is included in ik gateway
// idedge return true or false for general cases in build graph
// Is path being generated or loaded
// Tool transforms and planner.yaml config file
// Remember to switch useNumIK back to false if done using it


double ComputeManip(Eigen::VectorXd c, ikHandler& ik_handler){
    KDL::JntArray theta;
    KDL::Jacobian jac_kdl;
    theta = DFMapping::Eigen_to_KDLJoints(c);
    ik_handler.robot->Jac_KDL(theta,jac_kdl);
    Eigen::MatrixXd jac = DFMapping::KDLJacobian_to_Eigen(jac_kdl);
    double manip = (jac*jac.transpose()).determinant();
    return manip;
};

int main(int argc, char** argv){
    srand(time(0));

    ros::init(argc,argv,"pct_main");
    ros::NodeHandle main_handler;
    ros::Publisher cvrg_pub = main_handler.advertise<std_msgs::Bool>("cvrg_status",1000);

    timer main_timer;
    main_timer.start();
    double exec_time = 0;

    std::string csv_dir;
    csv_dir = ros::package::getPath("pct") + "/data/csv/";

    double resolution;
    if(!ros::param::get("/sampling_res",resolution)){
        std::cout<< "Unable to Obtain Sampling Resolution\n";
        return 0;
    }
    resolution *= (M_PI / 180);


    // std::vector<int> src_list = {2,4,1,3,6,8};
    // std::cout<< std::find( src_list.begin(), src_list.end(), 0 ) - src_list.begin() << "\n";
    // return 0;


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


    std::cout<< "Obtaining Tool TCPs\n";
    // TCP
    // Get transformation from ros parameter server
    // By default it takes the first tf as a nominal TCP
    std::string tool_name;
    if(!ros::param::get("/tool_name",tool_name))
        ROS_WARN("Unable to Obtain Tool name");
    std::vector<double> tf; tf.clear();
    tool_name = "/" + tool_name;
    if(!ros::param::get(tool_name + "/ff_T_tool",tf)){
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
    if (urPatch)
        ik_handler.enable_URikPatch();
    

    // Create Collision Checker
    WM::WM wm;
    wm.addRobot(robot_obj);
    std::string wp_path;
    if(!ros::param::get("/cvrg_file_paths/mesh_path",wp_path)){
        ROS_WARN("Unable to Obtain mesh path");
        return 1;
    }
    std::vector<double> tf_part(6);
    if(!ros::param::get("/cvrg_tf_param/world_T_part",tf_part)){
        ROS_WARN("Unable to Obtain part tf");
        return 1;
    }
    std::string toolstl_path;
    if(!ros::param::get(tool_name + "/tool_stl_coll_path",toolstl_path)){
        ROS_WARN("Unable to Obtain tool stl path");
        return 1;
    }
    tf_eigen<< tf_part[0],tf_part[1],tf_part[2],tf_part[3],tf_part[4],tf_part[5];
    Eigen::Matrix4d world_T_part = Eigen::Matrix4d::Identity();
    world_T_part.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
    world_T_part.block(0,3,3,1) = tf_eigen.segment(0,3);
    std::vector<Eigen::MatrixXd> zero_fk = robot.get_robot_FK_all_links( Eigen::MatrixXd::Ones(robot.NrOfJoints,1)*0 );
    // wm.addTool(toolstl_path);
    wm.prepareSelfCollisionPatch(zero_fk);
    wm.addWorkpiece(wp_path, world_T_part);

    Eigen::MatrixXd path;
    bool gen_path;
    ros::param::get("/gen_path",gen_path);
    if (gen_path)    
        // Generate Path
        path = gen_cvrg_plan();
    else{
        std::string path_file;
        ros::param::get("/cvrg_file_paths/path_file",path_file);
        path = load_plan(path_file); // Pre computed path file
    }

    int NumWaypoints = path.rows();


    // Obtain all the waypoints with tolerances applied at different depths
    // Note these points will be the transformed points in space
    std::vector<double> tolerances_vec; tolerances_vec.clear();
    if(!ros::param::get("/tolerances",tolerances_vec)){
        ROS_WARN("Unable to Obtain Waypoint Tolerances");
        return 1;
    }
    
    int choke_pts = 30;
    std::vector<int> idx;

    idx.push_back(floor(NumWaypoints/choke_pts));
    for (int i=1; i<choke_pts-1; ++i)
        idx.push_back( idx[i-1] + floor(NumWaypoints/choke_pts) );

    Eigen::MatrixXd tolerances(path.rows(),tolerances_vec.size());
    for (int i=0; i<path.rows(); ++i){
        for (int j=0; j<tolerances_vec.size(); ++j){
            tolerances(i,j) = tolerances_vec[j];
        }
    }
    tolerances *= (M_PI/180);

    double tol_constr;
    ros::param::get("/tol_constraint",tol_constr);

    for (int i=0; i<idx.size(); ++i)
        tolerances.row(idx[i]) = tolerances.row(idx[i])*tol_constr;
    
    // Add a piece of code here that randomly selects 10-20% of points
    // and makes the tolerances zero to make the problem tougher


    std::cout<< "Generating Search Samples....\n";
    main_timer.reset();
    std::vector<Eigen::MatrixXd> wpTol =  gen_wp_with_tolerance(tolerances,resolution, path );
    ROS_INFO( "Search Samples Generated....COMPUTE TIME: %f",  main_timer.elapsed() );

    // Geometric Filter Harness Initializer
    GeometricFilterHarness geo_filter;
    std::vector<Eigen::MatrixXd> ff_frames = 
    geo_filter.generate_flange_frames( wpTol,tcp_list );

    int fframes_cnt = 0, max_fframes = 0, min_fframes = 1e8;
    int id = 0;
    for (auto frames : ff_frames){
        fframes_cnt += frames.rows();
        // ROS_WARN_STREAM("Level: " << id << ". FF count: " << frames.rows());
        if (max_fframes < frames.rows())
            max_fframes = frames.rows();
        if (min_fframes > frames.rows())
            min_fframes = frames.rows();
        id++;
    }
    // return 0;

    // Get trajectory path
    std::string traj_path;
    ros::param::get("/cvrg_file_paths/joint_states",traj_path);
    Eigen::MatrixXd trajectory;
    Eigen::MatrixXd success_flags = Eigen::MatrixXd::Ones(path.rows(),1)*0;
    std::string success_flag_path;
    ros::param::get("/cvrg_file_paths/success_flags",success_flag_path);
    std::string opt_path_path = csv_dir + "opt_path.csv";

    std::vector<double> x(68);
    std::string x_path = ros::package::getPath("pct") + "/data/decision_variable/" + 
                            "random_nobias_opt_x.csv";
    // std::string x_path = ros::package::getPath("pct") + "/data/decision_variable/" + 
    //                         "pitr_nobias_opt_x.csv";
    std::vector<std::vector<double>> X = file_rw::file_read_vec(x_path);
    x = X[0];

    // for (int i=0; i<x.size(); ++i)
    //     x[i] = 0.5;

    ros::param::set("/decision_var",x);
    

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





    #ifdef SEQ_IK
    main_timer.reset();
    trajectory.resize(1,ik_handler.OptVarDim);
    trajectory.row(0) << ik_handler.init_guess.transpose();
    // Using Sequential IK
    std::cout<< "Solving sequential IK for multiple start points......\n";
    Eigen::VectorXd jt_config;
    if (ik_handler.solveIK(path.row(0))){ // IK for first row
        ik_handler.useNumIK = true;
        Eigen::MatrixXd strt_solutions = ik_handler.solution;
        // Eigen::MatrixXd strt_solutions = ik_handler.closest_sol;
        std::cout<< "No of solutions for first point: " << ik_handler.solution.cols() << "\n";
        std::cout<< "Number of starting points: " << strt_solutions.cols() << "\n";
        for (int i=0; i<strt_solutions.cols();++i){
            std::vector<Eigen::MatrixXd> fk_kdl = ik_handler.robot->get_robot_FK_all_links(strt_solutions.col(i));
            if(wm.inCollision( fk_kdl )){
                std::cout<< "In collision. Point index: " << i << "\n";
                continue;
            }
            ik_handler.init_guess = strt_solutions.col(i); // Initial guess as first point
            for (int j=0; j<path.rows(); ++j){
                if (ik_handler.solveIK(path.row(j))){
                    jt_config = ik_handler.closest_sol;
                    std::vector<Eigen::MatrixXd> fk_kdl = ik_handler.robot->get_robot_FK_all_links(jt_config);
                    if(!wm.inCollision( fk_kdl )){
                        ik_handler.init_guess = jt_config;
                        trajectory.conservativeResize(j+1,robot.NrOfJoints);
                        trajectory.row(j) = jt_config.transpose();
                        ik_handler.init_guess = jt_config;
                        success_flags(j,0) = 1;
                    }
                    else{
                        std::cout<< "IK not feasible. Point index: " << i << "\n";
                        break;
                        trajectory.conservativeResize(j+1,robot.NrOfJoints);
                        trajectory.row(j) = jt_config.transpose();
                        ik_handler.init_guess = jt_config;
                        success_flags(j,0) = 0;
                    }
                }
            }
            if (trajectory.rows()==path.rows())
                std::cout<< "Trajectory successfully found for sequential IK. Point Index: " << i <<"\n";
            else
                std::cout<< "Trajectory failure for sequential IK. Point Index: " << i <<"\n";
            file_rw::file_write(csv_dir+std::to_string(i)+".csv", trajectory);
        }
    }
    else{
        std::cout<< "First point not reachable\n";
    }
    ik_handler.useNumIK = false;
    ROS_INFO( "Sequential IK COMPUTE TIME: %f", main_timer.reset().elapsed() );
    #endif

    Eigen::VectorXi id_path;



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

    double path_cost;
    Eigen::MatrixXi attmps(NumWaypoints,2);

    #ifdef GRAPH_SEARCH

    #ifdef BASELINE
    // Graph Search
    Eigen::MatrixXi path_idx(NumWaypoints,1);
    // Eigen::MatrixXi tcp_idx(NumWaypoints,1);
    std::vector<node*> node_map;
    std::vector<Eigen::VectorXi> node_list;
    success_flags = Eigen::MatrixXd::Ones(NumWaypoints,1)*0;
    boost_graph graph;
    double search_time = 0;

    std::cout<< "\nGenerating Nodes\n";
    main_timer.reset();
    if(!gen_nodes(&ik_handler, &wm, &geo_filter, ff_frames, node_map, node_list, success_flags, attmps)){
        std::cout<< "Nodes could not be generated. No solution found\n";
        trajectory.resize(1,ik_handler.OptVarDim);
        trajectory.row(0) << ik_handler.init_guess.transpose();
    }
    else{
        exec_time += main_timer.elapsed();
        ROS_INFO( "Nodes generation COMPUTE TIME: %f", main_timer.elapsed() );
        #ifdef DEBUG_MODE_MAIN
        Eigen::MatrixXi reach_map = Eigen::MatrixXi::Ones(path.rows(),wpTol[0].rows())*0;
        for (int i=0; i<node_map.size(); ++i)
            reach_map(node_map[i]->depth,node_map[i]->index) = 1;
        #ifdef fileWrite
        file_rw::file_write(csv_dir+"reach_map.csv",reach_map);
        #endif // fileWrite
        #endif // DEBUG_MODE_MAIN

        std::vector<bool> root_connectivity;
        main_timer.reset();
        if(!build_graph(&ik_handler, ff_frames, &wm, &geo_filter,
                        node_map,node_list,&graph,root_connectivity)){
            std::cout<< "Edges could not be created. No solution found\n";
            trajectory.resize(1,ik_handler.OptVarDim);
            trajectory.row(0) << ik_handler.init_guess.transpose();
        }
        else{
            trajectory.resize(NumWaypoints,robot.NrOfJoints);
            // tcp_idx.resize(NumWaypoints,1);
            exec_time += main_timer.elapsed();
            ROS_INFO( "Graph generation COMPUTE TIME: %f", main_timer.elapsed() );
            
            // Change the start vertex in graph
            vertex_descriptor s = vertex(0, graph.g); graph.s = s;

            main_timer.reset();
            bool search_success = graph_searches::djikstra(&graph, id_path, path_cost);
            search_time += main_timer.elapsed();

            if(search_success){ // Get the shortest path to a leaf node in terms of node ids
                ROS_INFO_STREAM("Search Successful");
                // Generate Trajectory
                for(int k=0; k<id_path.size(); ++k)
                    trajectory.row(k) = node_map[id_path(k)]->jt_config.transpose();
                    // tcp_idx(k,0) = node_map[id_path(k)]->tcp_id;
                success_flags = Eigen::MatrixXd::Ones(NumWaypoints,1);
            }
            ROS_INFO_STREAM( "Source Config: " << node_map[id_path(0)]->jt_config.transpose() );
            ROS_INFO_STREAM( "Source Point Index: " << node_map[id_path(0)]->row_id );
            ROS_INFO_STREAM( "Source Waypoint: " << node_map[id_path(0)]->wp.transpose() );
            ROS_INFO_STREAM( "Sink Config: " << node_map[id_path(id_path.size()-1)]->jt_config.transpose() );
            ROS_INFO_STREAM( "Sink Point Index: " << node_map[id_path(id_path.size()-1)]->row_id );
            ROS_INFO_STREAM( "Sink Waypoint: " << node_map[id_path(id_path.size()-1)]->wp.transpose() );
        }
    }
    exec_time += search_time;
    std::cout<< "Time for graph search: " << search_time << "\n";
    #endif // Baseline ends











    #ifdef PCT_PLANNER
    std::vector<node*> node_map;
    std::vector<Eigen::VectorXi> node_list;
    success_flags = Eigen::MatrixXd::Ones(NumWaypoints,1)*0;
    boost_graph graph;
    Eigen::MatrixXi path_idx(NumWaypoints,1);
    Eigen::MatrixXi tcp_idx(NumWaypoints,1);
    Eigen::MatrixXd cost_hist;

    // Dummy root and leaf
    node* root_node = new node;
    root_node->id = 0;
    root_node->depth = -1;
    node_map.push_back(root_node);

    node* leaf_node = new node;
    leaf_node->id = 1;
    leaf_node->depth = ff_frames.size();
    node_map.push_back(leaf_node);

    main_timer.reset();
    ROS_INFO_STREAM("Building and Refining Graph");
    if (!BuildRefineGraph(&ik_handler, ff_frames, &wm, &geo_filter, 
                    node_map, node_list, &graph, cost_hist)){
        std::cout<< "Edges could not be created. No solution found\n";
        trajectory.resize(1,ik_handler.OptVarDim);
        trajectory.row(0) << ik_handler.init_guess.transpose();
    }
    else{
        trajectory.resize(NumWaypoints,ik_handler.OptVarDim);
        // tcp_idx.resize(NumWaypoints,1);
        double grph_time = main_timer.elapsed();
        exec_time += grph_time;
        ROS_INFO( "Graph generation COMPUTE TIME: %f", grph_time );
        
        // Change the start vertex in graph
        vertex_descriptor s = vertex(0, graph.g); graph.s = s;

        main_timer.reset();
        bool search_success = graph_searches::djikstra(&graph, id_path, path_cost);
        double search_time = main_timer.elapsed();
        exec_time += search_time;

        if(search_success){ // Get the shortest path to a leaf node in terms of node ids
            // Generate Trajectory
            for(int k=0; k<id_path.size(); ++k)
                trajectory.row(k) = node_map[id_path(k)]->jt_config.transpose();
                // tcp_idx(k,0) = node_map[id_path(k)]->tcp_id;
            success_flags = Eigen::MatrixXd::Ones(NumWaypoints,1);
        }
        ROS_INFO( "Search COMPUTE TIME: %f", search_time );

        ROS_INFO_STREAM( "Source Config: " << node_map[id_path(0)]->jt_config.transpose() );
        ROS_INFO_STREAM( "Source Point Index: " << node_map[id_path(0)]->row_id );
        ROS_INFO_STREAM( "Source Waypoint: " << node_map[id_path(0)]->wp.transpose() );
        ROS_INFO_STREAM( "Sink Config: " << node_map[id_path(id_path.size()-1)]->jt_config.transpose() );
        ROS_INFO_STREAM( "Sink Point Index: " << node_map[id_path(id_path.size()-1)]->row_id );
        ROS_INFO_STREAM( "Sink Waypoint: " << node_map[id_path(id_path.size()-1)]->wp.transpose() );
    }
    #endif


    #endif // Graph search
    
    // Evaluate trajectory cost
    double max_change = -std::numeric_limits<double>::infinity();
    int max_change_index;
    for (int i=0; i<trajectory.rows()-1;++i){
        Eigen::ArrayXd jt_diff = (trajectory.row(i+1) - trajectory.row(i)).transpose();
        if (jt_diff.abs().maxCoeff()>max_change){
            max_change = jt_diff.abs().maxCoeff();
            max_change_index = i+1;
        }
    }

    //Store the optimal path
    Eigen::MatrixXd opt_path;
    if (id_path.size()!=0){
        opt_path.resize(NumWaypoints,12);
        for (int i=0; i<id_path.size(); ++i){
            opt_path.row(i) = node_map[id_path(i)]->wp.transpose();
        }
    }

    #ifdef GRAPH_SEARCH
    std::cout<< "Total number of edges in graph: " << graph.no_edges << std::endl;
    std::cout<< "Total number of nodes in graph: " << graph.no_nodes << std::endl;
    #endif
    std::cout<< "Maximum Joint Angle Change in Trajectory: " << max_change*(180/M_PI) << 
        " degrees. Index: " << max_change_index << "\n";
    std::cout<< "Number of waypoints: " << NumWaypoints << "\n";
    std::cout<< "Number of TCPs: " << no_tcps << "\n";
    std::cout<< "Trajectory Cost: " << path_cost << "\n";
    std::cout<< "Execution Time: " << exec_time << "\n";
    #ifdef BASELINE
    int tot_attmpts = 0, max_attmps = 0; 
    int min_attmpts = 1e8;
    int max_val_attmps = 0; 
    int min_val_attmpts = 1e8;
    for (int i=0; i<attmps.rows(); ++i){
        tot_attmpts += attmps(i,0);
        if (max_attmps < attmps(i,0))
            max_attmps = attmps(i,0);
        if (min_attmpts > attmps(i,0))
            min_attmpts = attmps(i,0);

        if (max_val_attmps < attmps(i,1))
            max_val_attmps = attmps(i,1);
        if (min_val_attmpts > attmps(i,1))
            min_val_attmpts = attmps(i,1);
    }

    ROS_WARN_STREAM("Average FFrames per level: " << fframes_cnt / NumWaypoints << "\n" << 
                        "Max/Min FFrames: " << max_fframes << " / " << min_fframes);
    ROS_WARN_STREAM("Average Attempts per level: " << tot_attmpts / NumWaypoints << "\n" << 
                        "Max/Min Attempts: " << max_attmps << " / " << min_attmpts);
    ROS_WARN_STREAM("Average Valid Nodes per level: " << graph.no_nodes / NumWaypoints << "\n" << 
                        "Max/Min Valid Nodes: " << max_val_attmps << " / " << min_val_attmpts);

    #endif
    file_rw::file_write(traj_path,trajectory);
    file_rw::file_write(success_flag_path,success_flags);
    file_rw::file_write(opt_path_path,opt_path);
    std::cout<< "##############################################################\n";


    // SaveGraphStats(&graph, node_list, ff_frames);

    main_timer.end();


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