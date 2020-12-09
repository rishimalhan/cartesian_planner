///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <Eigen/Eigen>
#include <pct/timer.hpp>
#include <gen_utilities/file_rw.hpp>


// Borat
// Brotherrs grimbsi
//

int main(int argc, char** argv){
    srand(time(0));

    std::vector<std::string> test_cases;
    test_cases.push_back("roslaunch pct bootstrap.launch part:=bath_tub tool:=ferro_sander viz:=false");
    test_cases.push_back("roslaunch pct bootstrap.launch part:=step_slab tool:=ferro_sander  viz:=false");
    test_cases.push_back("roslaunch pct bootstrap.launch part:=boeing tool:=cam_sander_90 viz:=false");
    test_cases.push_back("roslaunch pct bootstrap.launch part:=gear_int tool:=ati viz:=false");

    ros::init(argc,argv,"x_eval");
    ros::NodeHandle main_handler;

    int ret_val;
    
    std::vector<double> x(68);
    std::string x_path = ros::package::getPath("pct") + "/data/decision_variable/" + 
                            "random_nobias_opt_x.csv";
    // std::string x_path = ros::package::getPath("pct") + "/data/decision_variable/" + 
    //                         "pitr_nobias_opt_x.csv";
    std::vector<std::vector<double>> X = file_rw::file_read_vec(x_path);
    x = X[0];
    ros::param::set("/decision_var",x);


    // ROS_WARN_STREAM("Feature Vector X: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);
    // Eigen::VectorXd cell_prob[2][2][2][2];
    // int ctr = 4;
    // for (int i=0; i<2; ++i){
    //     for (int j=0; j<2; ++j){
    //         for (int k=0; k<2; ++k){
    //             for (int l=0; l<2; ++l){
    //                 cell_prob[i][j][k][l].resize(4);
    //                 double sum = x[ctr] + x[ctr+1] + x[ctr+2] + x[ctr+3];
    //                 cell_prob[i][j][k][l](0) = x[ctr]/sum; ctr++;
    //                 cell_prob[i][j][k][l](1) = x[ctr]/sum; ctr++;
    //                 cell_prob[i][j][k][l](2) = x[ctr]/sum; ctr++;
    //                 cell_prob[i][j][k][l](3) = x[ctr]/sum; ctr++;
    //                 ROS_WARN_STREAM("Cell ID: " << i << j << k << l << ". Probability: " << 
    //                 cell_prob[i][j][k][l](0) << ", " << cell_prob[i][j][k][l](1) << ", " << 
    //                 cell_prob[i][j][k][l](2) << ", " << cell_prob[i][j][k][l](3));
    //             }
    //         }
    //     }
    // }
    // return 0;

    std::vector<std::vector<double>> history;
    
    timer main_timer;
    main_timer.start();
    ros::param::set("/decision_var",x);
    double time = 0;
    for (int i=0; i<test_cases.size(); ++i){
        main_timer.reset();
        std::string test_case = test_cases[i];
        ret_val = system(test_case.c_str());
        std::string cmd = "rosrun pct obj > /dev/null";
        ret_val = system(cmd.c_str());

        double obj_val;
        double exec_time;
        ros::param::get("/obj_val",obj_val);
        ros::param::get("/exec_time",exec_time);
        time += main_timer.elapsed();
        history.push_back( {obj_val,exec_time} );
    }

    for (int i=0; i<history.size(); ++i)
        ROS_WARN_STREAM("Objective Value: " << history[i][0] << ". Execution Time: " << history[i][1]);
    return 0;
}