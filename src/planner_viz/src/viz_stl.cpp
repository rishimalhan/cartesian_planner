#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <iostream>
#include <gen_utilities/file_rw.hpp>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <std_msgs/Bool.h>

bool doUpdate = true;

void cvrg_update(const std_msgs::Bool::ConstPtr& msg){
    doUpdate = msg->data;
    ROS_INFO("Subscribed to coverage update. Received: %d",doUpdate);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"viz_stl");
    ros::NodeHandle wp_handler;
    ros::Subscriber wp_sub = wp_handler.subscribe<std_msgs::Bool>("cvrg_plan",1000,cvrg_update);
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_ (new rviz_visual_tools::RvizVisualTools("world","/mesh"));
    visual_tools_->enableBatchPublishing(true);
    std::string stl_path;
    ros::param::get("/cvrg_file_paths/mesh_viz_path",stl_path);

    ros::Rate loop_rate(1000);
    while(ros::ok()){
        if (doUpdate){
            ROS_INFO("Updating Mesh on Display.....");
            visual_tools_->deleteAllMarkers();
            // Concatenate all paths in a pose array
            geometry_msgs::Pose curr_pose;
            curr_pose.position.x = 0;
            curr_pose.position.y = 0;
            curr_pose.position.z = 0;
            curr_pose.orientation.x = 0;
            curr_pose.orientation.y = 0;
            curr_pose.orientation.z = 0;
            curr_pose.orientation.w = 1;
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