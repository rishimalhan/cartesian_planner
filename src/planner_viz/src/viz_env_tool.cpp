#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <robot_utilities/transformation_utilities.hpp>
#include <ros/ros.h>
#include <Eigen/Eigen>

int main(int argc, char** argv){
    ros::init(argc,argv,"viz_env_tool");

    // Path to folder where collision stl is located
    std::string tool_name;
    if(!ros::param::get("/tool_name",tool_name)){
        ROS_WARN("Unable to Obtain Tool name");
        return 1;
    }
    std::string toolstl_path;
    if(!ros::param::get("/"+tool_name + "/tool_stl_viz_path",toolstl_path)){
        ROS_WARN("Unable to Obtain tool stl path");
        return 1;
    }

    // Mesh Path
    std::string wp_path;
    if(!ros::param::get("/cvrg_file_paths/mesh_path",wp_path)){
        ROS_WARN("Unable to Obtain mesh path");
        return 1;
    }
    std::vector<double> tf_part(6);
    Eigen::VectorXd tf_eigen(6);
    Eigen::VectorXd qt;

    // Visualization
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_ (new rviz_visual_tools::RvizVisualTools("world","/mesh"));
    visual_tools_->enableBatchPublishing(true);

    std::string stl_path;
    if(!ros::param::get("/cvrg_file_paths/mesh_viz_path",stl_path)){
        ROS_WARN("Unable to obtain Mesh Path");
        return 1;
    }


    ros::Rate loopRate(1000);
    while (ros::ok()){
        // Part to World tf: x,y,z,rx,ry,rz
        if(!ros::param::get("/cvrg_tf_param/world_T_part",tf_part)){
            ROS_WARN("Unable to Obtain part tf");
            return 1;
        }

        // Generate Eigen matrix from euler angles
        tf_eigen<< tf_part[0],tf_part[1],tf_part[2],tf_part[3],tf_part[4],tf_part[5];
        
        qt = rtf::eul2qt(tf_eigen.segment(3,3).transpose(),"XYZ").row(0);

        // Concatenate all paths in a pose array
        geometry_msgs::Pose part_pose;
        part_pose.position.x = tf_eigen(0);
        part_pose.position.y = tf_eigen(1);
        part_pose.position.z = tf_eigen(2);
        part_pose.orientation.x = qt(0);
        part_pose.orientation.y = qt(1);
        part_pose.orientation.z = qt(2);
        part_pose.orientation.w = qt(3);


        // Tool tf with respect to world which changes constantly.
        if(!ros::param::get("/world_T_tool",tf_part)){
            ROS_WARN("Unable to Obtain part tf");
            continue;
        }

        tf_eigen<< tf_part[0],tf_part[1],tf_part[2],tf_part[3],tf_part[4],tf_part[5];

        qt = rtf::eul2qt(tf_eigen.segment(3,3).transpose(),"XYZ").row(0);

        // Concatenate all paths in a pose array
        geometry_msgs::Pose curr_pose;
        curr_pose.position.x = tf_eigen(0);
        curr_pose.position.y = tf_eigen(1);
        curr_pose.position.z = tf_eigen(2);
        curr_pose.orientation.x = qt(0);
        curr_pose.orientation.y = qt(1);
        curr_pose.orientation.z = qt(2);
        curr_pose.orientation.w = qt(3);

        visual_tools_->deleteAllMarkers();

        if(!visual_tools_->publishMesh(part_pose, stl_path, rviz_visual_tools::GREY, 1.0))
            std::cout<< "Publishing Mesh Failed\n";   
        if(!visual_tools_->publishMesh(curr_pose, toolstl_path, rviz_visual_tools::GREY, 1.0))
                std::cout<< "Publishing Tool Failed\n";   

        visual_tools_->trigger();
    }




    return 0;
}