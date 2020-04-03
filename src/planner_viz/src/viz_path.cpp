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
    ros::init(argc,argv,"viz_path");
    ros::NodeHandle wp_handler;
    ros::Subscriber wp_sub = wp_handler.subscribe<std_msgs::Bool>("cvrg_plan",1000,cvrg_update);
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_ (new rviz_visual_tools::RvizVisualTools("world","/path"));
    visual_tools_->enableBatchPublishing(true);
    std::string path_path;
    ros::param::get("/cvrg_file_paths/cvrg_path",path_path);

    ros::Rate loop_rate(1000);
    while(ros::ok()){
        if (doUpdate){
            ROS_INFO("Updating Path on Display.....");
            visual_tools_->deleteAllMarkers();
            Eigen::MatrixXd path = file_rw::file_read_mat(path_path);
            // Concatenate all paths in a pose array
            std::vector<geometry_msgs::Point> path_points; path_points.clear();
            geometry_msgs::Point curr_pt;
            for (int i=0; i< path.rows();++i){
                curr_pt.x = path(i,0);
                curr_pt.y = path(i,1);
                curr_pt.z = path(i,2);
                path_points.push_back(curr_pt);
            }
            if(!visual_tools_->publishPath(path_points, rviz_visual_tools::GREEN, 0.01, "1"))
                std::cout<< "Publishing Axis Failed\n";   
            visual_tools_->trigger();
            doUpdate = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}