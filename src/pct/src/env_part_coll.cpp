///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#include <robot_utilities/world_manager.hpp>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <robot_utilities/transformation_utilities.hpp>


int main(int argc, char** argv){
    ros::init(argc,argv,"toolColl");
    ros::NodeHandle main_handler;

    std::vector<double> tf_part(6);
    Eigen::VectorXd tf_eigen(6);
    // Mesh Path
    std::string wp_path;
    if(!ros::param::get("/cvrg_file_paths/mesh_path",wp_path)){
        ROS_WARN("Unable to Obtain mesh path");
        return 1;
    }
    // Part to World tf: x,y,z,rx,ry,rz
    if(!ros::param::get("/cvrg_tf_param/world_T_part",tf_part)){
        ROS_WARN("Unable to Obtain part tf");
        return 1;
    }
    // Path to folder where collision stl is located
    std::string tool_name;
    if(!ros::param::get("/tool_name",tool_name)){
        ROS_WARN("Unable to Obtain Tool name");
        return 1;
    }
    std::string toolstl_folder;
    if(!ros::param::get("/"+tool_name + "/tool_stl_coll_folder",toolstl_folder)){
        ROS_WARN("Unable to Obtain tool collision stl folder");
        return 1;
    }
    // Generate Eigen matrix from euler angles
    tf_eigen<< tf_part[0],tf_part[1],tf_part[2],tf_part[3],tf_part[4],tf_part[5];
    Eigen::Matrix4d world_T_part = Eigen::Matrix4d::Identity();
    world_T_part.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
    world_T_part.block(0,3,3,1) = tf_eigen.segment(0,3);

    // Create Collision Checker
    WM::WM wm;
    // Add tool as a robot
    wm.addRobot(toolstl_folder);
    // Add the workpiece
    wm.addWorkpiece(wp_path, world_T_part);
    // Any other env variables can be added here



    ros::Rate loopRate(1000);
    while (ros::ok()){
        // Tool tf with respect to world which changes constantly.
        if(!ros::param::get("/world_T_tool",tf_part)){
            ROS_WARN("Unable to Obtain part tf");
            continue;
        }
        tf_eigen<< tf_part[0],tf_part[1],tf_part[2],tf_part[3],tf_part[4],tf_part[5];
        Eigen::Matrix4d world_T_tool = Eigen::Matrix4d::Identity();
        world_T_tool.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
        world_T_tool.block(0,3,3,1) = tf_eigen.segment(0,3);
        // std vector holding the tool to world transform
        std::vector<Eigen::MatrixXd> tool_tf(1);
        tool_tf[0] = world_T_tool;
        std::cout<< "Collision Status: " << wm.inCollision(tool_tf) << "\n";
        loopRate.sleep();
    }

    return 0;
}