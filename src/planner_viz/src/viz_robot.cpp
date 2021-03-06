#include <ros/ros.h>
#include <iostream>
#include <robot_utilities/file_rw.hpp>
#include <Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <robot_utilities/ikHandler.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>

bool doUpdate = true;

Eigen::MatrixXd IncreaseTrajResolution( Eigen::MatrixXd traj ){
    return traj;
    // High resolution linear interpolation of trajectory to check if traj is actually PC.
    int no_samples = 10;
    Eigen::MatrixXd upd_traj;
    int counter = 0;
    if (traj.rows()==1)
        return traj;
    for (int i=0; i<traj.rows()-1; ++i){
        Eigen::VectorXd dt = (traj.row(i+1)-traj.row(i))/no_samples;
        for (int j=0; j<no_samples; ++j){
            upd_traj.conservativeResize(counter+1,traj.cols());
            upd_traj.row(counter) = traj.row(i) + (j*dt).transpose();
            counter++;
        }
    }
    return upd_traj;
}

void cvrg_update(const std_msgs::Bool::ConstPtr& msg){
    doUpdate = msg->data;
    ROS_INFO("Subscribed to coverage update. Received: %d",doUpdate);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"viz_robot");
    ros::NodeHandle rob_handler;

    // Fire up all the publisher to publish the joint state values
    ros::Publisher config_publ = rob_handler.advertise<sensor_msgs::JointState>( "joint_states", 1000 );
    ros::Subscriber wp_sub = rob_handler.subscribe<std_msgs::Bool>("cvrg_status",1000,cvrg_update);

    std::string traj_path;
    if(!ros::param::get("/cvrg_file_paths/joint_states",traj_path))
        ROS_INFO("Unable to obtain robot Trajectory");
    Eigen::MatrixXd traj = IncreaseTrajResolution(file_rw::file_read_mat(traj_path));

    sensor_msgs::JointState joint_config;
    joint_config.name.resize(traj.cols());
    for (int i=0; i<traj.cols(); i++)
        // joint_config.name[i] = "iiwa_joint_" + std::to_string(i+1);
        joint_config.name[i] = "joint_" + std::to_string(i+1);
    
    joint_config.position.resize(traj.cols());

    ros::Rate loop_rate(1000);
    int frequency;
    while(ros::ok()){
        for (int i=0; i<traj.rows();++i){
            if(!ros::param::get("/viz_param/rob_speed",frequency))
                ROS_INFO("Unable to obtain robot rate");
            loop_rate = ros::Rate(frequency);
            if (doUpdate){
                ROS_INFO("Updating Trajectory on Display.....");
                ros::param::get("/cvrg_file_paths/joint_states",traj_path);
                traj = IncreaseTrajResolution(file_rw::file_read_mat(traj_path));
                ROS_INFO("Size of Trajectory: %d", (int)traj.rows());
                doUpdate = false;
                break;
            }
            for (int j=0; j<traj.cols();++j)
                joint_config.position[j] = traj(i,j);
            joint_config.header.stamp = ros::Time::now();
            config_publ.publish(joint_config);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}