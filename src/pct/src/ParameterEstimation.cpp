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


int main(int argc, char** argv){
    srand(time(0));

    std::vector<std::string> test_cases;
    std::vector<double> opt_vals;
    test_cases.push_back("roslaunch pct bootstrap.launch part:=bath_tub tool:=ferro_sander viz:=false");
    test_cases.push_back("roslaunch pct bootstrap.launch part:=step_slab tool:=ferro_sander  viz:=false");
    test_cases.push_back("roslaunch pct bootstrap.launch part:=boeing tool:=cam_sander_90 viz:=false");
    test_cases.push_back("roslaunch pct bootstrap.launch part:=gear_int tool:=ati viz:=false");
    // test_cases.push_back("roslaunch pct bootstrap.launch part:=bath_tub tool:=ferro_sander viz:=false");

    opt_vals.push_back(3.12);
    opt_vals.push_back(6.25);
    opt_vals.push_back(66.03);
    opt_vals.push_back(10.46);
    // opt_vals.push_back();

    ros::init(argc,argv,"param_est");
    ros::NodeHandle main_handler;

    int ret_val;
    
    std::vector<double> x(68);
    std::vector<std::vector<double>> history;
    std::vector<std::vector<double>> dec_vars;

    timer main_timer;
    main_timer.start();
    
    for (int itr=0; itr<100; ++itr){
        double multi_obj_val = 0;
        for (int i=0; i<x.size(); ++i)
            x[i] = (double) std::rand() / RAND_MAX;
            // x[i] = 0.5;
        dec_vars.push_back(x);
        ros::param::set("/decision_var",x);
        double time = 0;
        main_timer.reset();
        for (int i=0; i<test_cases.size(); ++i){
            std::string test_case = test_cases[i];
            double opt_val = opt_vals[i];
            ret_val = system(test_case.c_str());
            std::string cmd = "rosrun pct obj > /dev/null";
            ret_val = system(cmd.c_str());

            double obj_val;
            ros::param::get("/obj_val",obj_val);
            bool check_nan = std::isnan(obj_val);
            if (check_nan){
                multi_obj_val = std::numeric_limits<double>::infinity();
                break;
            }
            multi_obj_val += (obj_val / opt_val);
            time += main_timer.elapsed();
        }
        history.push_back( {multi_obj_val,time} );
    }

    double obj_val;
    // Get X file path
    std::string x_path = ros::package::getPath("pct") + "/data/decision_variable/" + 
                            "pitr_nobias_opt_x.csv";
    double max_obj = std::numeric_limits<float>::infinity();
    for (int i=0; i<history.size(); ++i){
        obj_val = history[i][0]/10 + history[i][1]/250;
        ROS_WARN_STREAM("Objective Value: " << history[i][0] << ". Time: " << history[i][1]);
        std::vector<double> X = dec_vars[i];
        if (max_obj>obj_val){
            max_obj = obj_val;
            file_rw::file_write(x_path,X);
        }
    }
    return 0;
}