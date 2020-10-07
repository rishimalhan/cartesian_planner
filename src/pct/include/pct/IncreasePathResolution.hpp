///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __INCREASEPATHRESOLUTION__HPP__
#define __INCREASEPATHRESOLUTION__HPP__

#include <robot_utilities/ikHandler.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <Eigen/Eigen>

Eigen::MatrixXd IncreasePathResolution(Eigen::MatrixXd path, int no_samples){
    if (path.rows()==1)
        return path;

    int counter = 0;
    Eigen::MatrixXd upd_path;
    for (int i=0; i<path.rows()-1; ++i){
        Eigen::VectorXd dx = (path.row(i+1)-path.row(i)).transpose()/no_samples;
        for (int j=0; j<no_samples; ++j){
            upd_path.conservativeResize(counter+1,path.cols());
            upd_path.row(counter) = path.row(i) + (dx*j).transpose();
            counter++;
        }
    }
    return upd_path;
};

#endif