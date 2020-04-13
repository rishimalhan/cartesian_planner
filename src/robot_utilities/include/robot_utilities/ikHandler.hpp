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
#include <robot_utilities/SerialLink_Manipulator.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>
#include <robot_utilities/ikfast_gateway.hpp>


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
    KDL::Frame FK_tcp;
    Eigen::VectorXd target;
    Eigen::MatrixXd world_T_robBase;
    Eigen::MatrixXd robBase_T_world;
public:
    bool urIKPatch;
    Eigen::VectorXd closest_sol;
    Eigen::VectorXd jt_ul;
    Eigen::VectorXd jt_ll;
    SerialLink_Manipulator::SerialLink_Manipulator* robot;
    Eigen::MatrixXd solution;
    bool useNumIK = false;
    int OptVarDim;  // Decision variable dimension
    double f_val;
    bool status;
    Eigen::VectorXd init_guess;
    ikHandler(SerialLink_Manipulator::SerialLink_Manipulator*);
    ~ikHandler();
    void tf_target(Eigen::VectorXd&, const Eigen::MatrixXd&);
    double obj_func(const std::vector<double>&, std::vector<double>&);
    void apply_URikPatch(Eigen::MatrixXd &);
    double err_func(const std::vector<double>&);
    bool solveIK(Eigen::VectorXd);
    void enable_URikPatch();
    void disable_URikPatch();
    void setTcpFrame(const Eigen::MatrixXd&);
};


#endif