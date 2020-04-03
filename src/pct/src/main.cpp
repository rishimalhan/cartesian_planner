#include <iostream>
#include <pct/gen_cvrg_plan.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <pct/genTolWp.hpp>
#include <gen_utilities/file_rw.hpp>
#include <gen_utilities/transformation_utilities.hpp>
#include <pct/jacQP.hpp>

int main(int argc, char** argv){
    // int b = 2;
    // long int sum = 1;
    // for (int i=1; i<=10; ++i)
    //     sum += pow(b,i);
    // std::cout<< sum << "\n";
    // return 0;

    ros::init(argc,argv,"main");
    ros::NodeHandle main_handler;
    ros::Publisher cvrg_pub = main_handler.advertise<std_msgs::Bool>("cvrg_plan",1000);
    Eigen::MatrixXd path = gen_cvrg_plan();
    std_msgs::Bool msg;
    msg.data = true;
    ROS_INFO("Publishing Message");
    ros::Rate loop_rate(1000);
    for(int i=0; i<1000; ++i){
        cvrg_pub.publish(msg);
        loop_rate.sleep();
    }

    // Obtain all the waypoints with tolerances applied at different depths
    double resolution = 10*M_PI / 180;
    double angle = 90*M_PI / 180;
    std::cout<< "Generating Search Samples....\n";
    Eigen::MatrixXd tolerances = Eigen::MatrixXd::Ones(path.rows(),1)*angle;
    std::vector<Eigen::MatrixXd> wpTol =  gen_wp_with_tolerance(tolerances,resolution, path );

    // Call the Search
    solveQP();
    return 0;
}