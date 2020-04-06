#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <iostream>
#include <gen_utilities/file_rw.hpp>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <gen_utilities/transformation_utilities.hpp>

bool doUpdate = true;

void cvrg_update(const std_msgs::Bool::ConstPtr& msg){
    doUpdate = msg->data;
    ROS_INFO("Subscribed to coverage update. Received: %d",doUpdate);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"viz_stl");
    ros::NodeHandle wp_handler;
    ros::Subscriber wp_sub = wp_handler.subscribe<std_msgs::Bool>("cvrg_status",1000,cvrg_update);
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_ (new rviz_visual_tools::RvizVisualTools("world","/mesh"));
    visual_tools_->enableBatchPublishing(true);
    std::string stl_path;
    
    ros::Rate loop_rate(1000);
    while(ros::ok()){
        if (doUpdate){
            ROS_INFO("Updating Mesh on Display.....");
            ros::param::get("/cvrg_file_paths/mesh_viz_path",stl_path);
            visual_tools_->deleteAllMarkers();
            // Get Part Transform
            std::vector<double> tf;
            ros::param::get("/cvrg_tf_param/world_T_part",tf);
            Eigen::VectorXd tf_eigen(6);
            tf_eigen<< tf[0],tf[1],tf[2],tf[3],tf[4],tf[5];
            Eigen::VectorXd qt = rtf::eul2qt(tf_eigen.segment(3,3).transpose(),"XYZ").row(0);

            // Concatenate all paths in a pose array
            geometry_msgs::Pose curr_pose;
            curr_pose.position.x = tf_eigen(0);
            curr_pose.position.y = tf_eigen(1);
            curr_pose.position.z = tf_eigen(2);
            curr_pose.orientation.x = qt(0);
            curr_pose.orientation.y = qt(1);
            curr_pose.orientation.z = qt(2);
            curr_pose.orientation.w = qt(3);
            if(!visual_tools_->publishMesh(curr_pose, stl_path, rviz_visual_tools::GREY, 1.0))
                std::cout<< "Publishing Axis Failed\n";   
            visual_tools_->trigger();
            doUpdate = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}