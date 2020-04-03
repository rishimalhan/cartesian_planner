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
    ros::init(argc,argv,"viz_wp");
    ros::NodeHandle wp_handler;
    ros::Subscriber wp_sub = wp_handler.subscribe<std_msgs::Bool>("cvrg_plan",1000,cvrg_update);
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_ (new rviz_visual_tools::RvizVisualTools("world","/wp"));
    visual_tools_->enableBatchPublishing(true);
    std::string path_path;
    ros::param::get("/cvrg_file_paths/cvrg_path",path_path);

    ros::Rate loop_rate(1000);
    while(ros::ok()){
        if (doUpdate){
            ROS_INFO("Updating Waypoints on Display.....");
            visual_tools_->deleteAllMarkers();
            Eigen::Matrix3d rot_mat;
            Eigen::MatrixXd path = file_rw::file_read_mat(path_path);
            // Concatenate all paths in a pose array
            geometry_msgs::Pose curr_pose;
            for (int i=0; i< path.rows();++i){
                rot_mat << path.row(i).block(0,3,1,3).transpose(), 
                            path.row(i).block(0,6,1,3).transpose(),
                            path.row(i).block(0,9,1,3).transpose();
                Eigen::Quaterniond q(rot_mat);
                curr_pose.position.x = path(i,0);
                curr_pose.position.y = path(i,1);
                curr_pose.position.z = path(i,2);
                curr_pose.orientation.x = q.x();
                curr_pose.orientation.y = q.y();
                curr_pose.orientation.z = q.z();
                curr_pose.orientation.w = q.w();
                if(!visual_tools_->publishAxis(curr_pose, 0.1,0.01, std::to_string(i)))
                    std::cout<< "Publishing Axis Failed\n";   
            }
            visual_tools_->trigger();
            doUpdate = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}