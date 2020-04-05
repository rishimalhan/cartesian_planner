// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function is an implementation of a quadratic program for finding an inverse kinematics solution for a given target,
//  previous configuration and constraints imposed.
#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <iostream>
#include <gen_utilities/SerialLink_Manipulator.hpp>
#include <gen_utilities/Data_Format_Mapping.hpp>


// Declarations
// Class numIK starts here
class numIK{
private:
    // Optimization Variables
    nlopt::opt optimizer;
    nlopt::algorithm alg_type;
    int OptVarDim;  // Decision variable dimension
    double optXtolRel;  // Relative tolerance for stopping condition
    double gradH;   // Finite difference step size
    double f_val;
    bool frst_pt;
    bool status;
    std::vector<double> OptVarlb;   // Lower Bounds
    std::vector<double> OptVarub;   // Upper Bounds
    SerialLink_Manipulator::SerialLink_Manipulator* robot;
    KDL::Frame FK_tcp;
    Eigen::VectorXd target;
public:
    numIK(SerialLink_Manipulator::SerialLink_Manipulator*);
    ~numIK();
    double obj_func(const std::vector<double>&, std::vector<double>&);
    double err_func(const std::vector<double>&);
    Eigen::MatrixXd solveIK(const Eigen::VectorXd&, const Eigen::VectorXd&);
};




// Definitions




// defualt nlopt fuctions begin
// Error function only minimizes the obejective. So for max use negative.
double err_func_gateway(const std::vector<double>& x, std::vector<double>& grad, void* data) 
{
    // Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.
    numIK *gateway = (numIK *) data;
    return gateway->obj_func(x,grad);
}
// defualt nlopt fuctions end


numIK::numIK(SerialLink_Manipulator::SerialLink_Manipulator* _robot){
    robot = _robot;
    OptVarDim = robot->NrOfJoints;

    //choose optimizer
    // alg_type = nlopt::LN_NEWUOA;
    // alg_type = nlopt::LN_NEWUOA_BOUND;
    // alg_type = nlopt::LD_MMA;
    // alg_type = nlopt::LD_TNEWTON_PRECOND_RESTART;
    // alg_type = nlopt::LN_BOBYQA;
    // alg_type = nlopt::LN_COBYLA;
    alg_type = nlopt::LD_SLSQP;
    // alg_type = nlopt::LD_LBFGS;
    // alg_type = nlopt::GN_ISRES;
    optXtolRel = 1e-4;
    gradH = 1e-8;
    // Lower and Upper Bounds for joints
    OptVarlb.resize(OptVarDim);
    OptVarub.resize(OptVarDim);
    for (int i=0; i<OptVarDim; ++i){
        OptVarlb[i] = robot->Joints_ll(i);
        OptVarub[i] = robot->Joints_ul(i);
    }
    // Determine dimension of all the constraints. If constraints are applied successively,
    // Set this to max dimension and keep flag in constr_func which will keep 
    // non-enforced constraints zero.
    optimizer = nlopt::opt(alg_type, OptVarDim);
    optimizer.set_xtol_rel(optXtolRel);
    optimizer.set_min_objective(err_func_gateway, this);
    optimizer.set_lower_bounds(OptVarlb);
    optimizer.set_upper_bounds(OptVarub);
    optimizer.set_ftol_rel(1e-8);   // Tolerance in objective function change.
    optimizer.set_maxeval(2000);
    status = false;
    target.resize(12);
}

numIK::~numIK() {};

double numIK::obj_func(const std::vector<double>& x, std::vector<double>& grad)
{
    double err = err_func(x);
    if (!grad.empty()) {
        std::vector<double> xph = x;
        for (uint i=0; i < x.size(); ++i)
        {
            xph[i] += gradH;
            grad[i] = (err_func(xph)-err)/gradH;
            xph[i] -= gradH;
        }
    }    
    return err;
}

double numIK::err_func(const std::vector<double>& x)
{
    KDL::JntArray joint_config = DFMapping::STDvector_to_KDLJoints(x);
    robot->FK_KDL_TCP(joint_config, FK_tcp);
    return ( pow(target(0)-FK_tcp(0,3),2) + pow(target(1)-FK_tcp(1,3),2) + pow(target(2)-FK_tcp(2,3),2) + // Translation Error
              pow( 1 - (target(3)*FK_tcp(0,0) + target(4)*FK_tcp(1,0) + target(5)*FK_tcp(2,0)) , 2 ) + //bx
               pow( 1 - (target(6)*FK_tcp(0,1) + target(7)*FK_tcp(1,1) + target(8)*FK_tcp(2,1)) , 2 ) +    //by
               pow( 1 - (target(9)*FK_tcp(0,2) + target(10)*FK_tcp(1,2) + target(11)*FK_tcp(2,2)) , 2 ) ); //bz
};





//////////////////////// MAIN FUNCTION //////////////////////////////////////////////////
Eigen::MatrixXd numIK::solveIK(const Eigen::VectorXd& init_guess, const Eigen::VectorXd& _target){
    target = _target;
    Eigen::MatrixXd solution(OptVarDim,1);
    
    std::vector<double> iterator(OptVarDim);
    for (int i=0; i<OptVarDim; ++i)
        iterator[i] = init_guess(i,0);

    // NLopt routine
    bool optSuccess = false;
    try{
        
        nlopt::result result = optimizer.optimize(iterator, f_val);
        optSuccess = true;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    if (!optSuccess || f_val > 1e-4)
        status = false;

    for (int i=0; i < OptVarDim; i++)
        solution(i,0) = iterator[i];
    return solution;
}