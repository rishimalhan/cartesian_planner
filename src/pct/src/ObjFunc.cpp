///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
#include <pct/graph_description.hpp>
#include <pct/graph_searches.hpp>
#include <pct/timer.hpp>
#include <pct/IncreasePathResolution.hpp>
#include <pct/geometric_filter.h>
#include <gen_utilities/utilities.hpp>
#include <unordered_set>
#include <pct/PCTplanner.hpp>
#include <random>




int main(int argc, char** argv){
    srand(time(0));

    ros::init(argc,argv,"obj_func");
    ros::NodeHandle main_handler;

    std::string csv_dir;
    csv_dir = ros::package::getPath("pct") + "/data/csv/";

    double resolution;
    if(!ros::param::get("/sampling_res",resolution)){
        std::cout<< "Unable to Obtain Sampling Resolution\n";
        return 0;
    }
    resolution *= (M_PI / 180);

    // ROBOT IRB2600
    Eigen::MatrixXd init_guess(6,1);
    init_guess << 0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463;
    std::string rob_base_link = "base_link";
    std::string rob_tip_link = "tool0";
    std::string urdf_path = ros::package::getPath("robot_utilities") + "/urdf/abb_irb2600/irb2600_12_165.urdf";
    std::string robot_obj = ros::package::getPath("robot_utilities") + "/rob_objs/abb_irb2600/";
    KDL::Frame base_frame = KDL::Frame::Identity();
    bool urPatch = false;


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
    // Add 7 0 tolerance points at 25% intervals to constraint problem
    int idx1 = floor(NumWaypoints/8);
    int idx2 = idx1 + floor(NumWaypoints/8);
    int idx3 = idx2 + floor(NumWaypoints/8);
    int idx4 = idx3 + floor(NumWaypoints/8);
    int idx5 = idx4 + floor(NumWaypoints/8);
    int idx6 = idx5 + floor(NumWaypoints/8);
    int idx7 = idx6 + floor(NumWaypoints/8);
    
    Eigen::MatrixXd tolerances(path.rows(),tolerances_vec.size());
    for (int i=0; i<path.rows(); ++i){
        for (int j=0; j<tolerances_vec.size(); ++j){
            tolerances(i,j) = tolerances_vec[j];
        }
    }
    tolerances *= (M_PI/180);

    double tol_constr;
    ros::param::get("/tol_constraint",tol_constr);

    tolerances.row(idx1) = tolerances.row(idx1)*tol_constr;
    tolerances.row(idx2) = tolerances.row(idx2)*tol_constr;
    tolerances.row(idx3) = tolerances.row(idx3)*tol_constr;
    tolerances.row(idx4) = tolerances.row(idx4)*tol_constr;
    tolerances.row(idx5) = tolerances.row(idx5)*tol_constr;
    tolerances.row(idx6) = tolerances.row(idx6)*tol_constr;
    tolerances.row(idx7) = tolerances.row(idx7)*tol_constr;

    



    std::cout<< "Generating Search Samples....\n";
    std::vector<Eigen::MatrixXd> wpTol =  gen_wp_with_tolerance(tolerances,resolution, path );

    // Geometric Filter Harness Initializer
    GeometricFilterHarness geo_filter;
    std::vector<Eigen::MatrixXd> ff_frames = 
    geo_filter.generate_flange_frames( wpTol,tcp_list );

    double cost = 0;
    int no_sols = 0;
    double exec_time = 0;
    for (int itr=0; itr<10; itr++){
        std::vector<node*> node_map;
        std::vector<Eigen::VectorXi> node_list;
        boost_graph graph;
        double no_strt_nodes;

        timer exec_timer;
        exec_timer.start();
        ROS_INFO_STREAM("Building and Refining Graph");
        if (!BuildRefineGraph(&ik_handler, ff_frames, &wm, &geo_filter, node_map, node_list, &graph)){
            continue;
        }
        else{
            exec_time += exec_timer.elapsed();
            // Search
            Eigen::VectorXi strt_nodes = node_list[0];
            no_strt_nodes = strt_nodes.size();
            double lowest_cost = std::numeric_limits<double>::infinity();
            // Run Search For Multiple Start Configs
            for (int i=0; i<strt_nodes.size(); ++i){
                int root_node = strt_nodes(i);
                Eigen::VectorXd root_config = node_map[root_node]->jt_config;
                // Change the start vertex in graph
                vertex_descriptor s = vertex(root_node, graph.g); graph.s = s;
                Eigen::VectorXi id_path;
                bool search_success = graph_searches::djikstra(&graph,id_path);

                if(search_success){ // Get the shortest path to a leaf node in terms of node ids
                    // Generate Trajectory
                    Eigen::MatrixXd curr_traj(NumWaypoints, robot.NrOfJoints);
                    for(int k=0; k<id_path.size(); ++k)
                        curr_traj.row(k) = node_map[id_path(k)]->jt_config.transpose();
                    // Evaluate trajectory cost
                    double path_cost = 0;
                    for (int k=0; k<curr_traj.rows()-1;++k){
                        Eigen::ArrayXd jt_diff = (curr_traj.row(k+1) - curr_traj.row(k)).transpose();
                        path_cost += jt_diff.abs().maxCoeff();
                    }
                    if (path_cost<lowest_cost)
                        lowest_cost = path_cost;
                }
            }
            cost += lowest_cost;
            no_sols++;
        }
    }
    ros::param::set("/obj_val",cost / no_sols);
    ros::param::set("/exec_time",exec_time / no_sols);
    return 0;
}