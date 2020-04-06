#include <iostream>
#include <Eigen/Eigen>
#include <gen_utilities/SerialLink_Manipulator.hpp>
#include <gen_utilities/Data_Format_Mapping.hpp>

double computeTrajErr(SerialLink_Manipulator::SerialLink_Manipulator* robot, Eigen::MatrixXd& trajectory, 
                Eigen::MatrixXd& path){
    // Version-1. Solve FK and Check overall error
};