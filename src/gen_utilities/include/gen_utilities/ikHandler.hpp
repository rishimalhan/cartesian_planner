// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function is an implementation of a quadratic program for finding an inverse kinematics solution for a given target,
//  previous configuration and constraints imposed.
#ifndef __ikHandler_HPP__
#define __ikHandler_HPP__

#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <iostream>
#include <gen_utilities/SerialLink_Manipulator.hpp>
#include <gen_utilities/Data_Format_Mapping.hpp>
#include <gen_utilities/ikfast_gateway.hpp>


// Declarations
// Class ikHandler starts here
class ikHandler{
private:
    // Optimization Variables
    nlopt::opt optimizer;
    nlopt::algorithm alg_type;
    double optXtolRel;  // Relative tolerance for stopping condition
    double gradH;   // Finite difference step size
    std::vector<double> OptVarlb;   // Lower Bounds
    std::vector<double> OptVarub;   // Upper Bounds
    SerialLink_Manipulator::SerialLink_Manipulator* robot;
    KDL::Frame FK_tcp;
    Eigen::VectorXd target;
public:
    Eigen::MatrixXd solution;
    bool useNumIK = false;
    int OptVarDim;  // Decision variable dimension
    double f_val;
    bool status;
    Eigen::VectorXd init_guess;
    ikHandler(SerialLink_Manipulator::SerialLink_Manipulator*);
    ~ikHandler();
    double obj_func(const std::vector<double>&, std::vector<double>&);
    double err_func(const std::vector<double>&);
    bool solveIK(const Eigen::VectorXd&);
};


#endif