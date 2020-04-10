#include <gen_utilities/ikHandler.hpp>

// Definitions
// defualt nlopt fuctions begin
// Error function only minimizes the obejective. So for max use negative.
double err_func_gateway(const std::vector<double>& x, std::vector<double>& grad, void* data) 
{
    // Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.
    ikHandler *gateway = (ikHandler *) data;
    return gateway->obj_func(x,grad);
}
// defualt nlopt fuctions end


ikHandler::ikHandler(SerialLink_Manipulator::SerialLink_Manipulator* _robot){
    robot = _robot;
    OptVarDim = robot->NrOfJoints;
    init_guess.resize(OptVarDim,1);
    // Default initial guess
    for (int i=0; i<OptVarDim; ++i)
        init_guess(i,0) = 0;

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
    std::cout<< "#########################################################################\nikHandler initialization complete\n" <<
    "#########################################################################\n";
}

ikHandler::~ikHandler() {};

double ikHandler::obj_func(const std::vector<double>& x, std::vector<double>& grad)
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

double ikHandler::err_func(const std::vector<double>& x)
{
    KDL::JntArray joint_config = DFMapping::STDvector_to_KDLJoints(x);
    robot->FK_KDL_TCP(joint_config, FK_tcp);
    return ( pow(target(0)-FK_tcp(0,3),2) + pow(target(1)-FK_tcp(1,3),2) + pow(target(2)-FK_tcp(2,3),2) + // Translation Error
              pow( 1 - (target(3)*FK_tcp(0,0) + target(4)*FK_tcp(1,0) + target(5)*FK_tcp(2,0)) , 2 ) + //bx
               pow( 1 - (target(6)*FK_tcp(0,1) + target(7)*FK_tcp(1,1) + target(8)*FK_tcp(2,1)) , 2 ) +    //by
               pow( 1 - (target(9)*FK_tcp(0,2) + target(10)*FK_tcp(1,2) + target(11)*FK_tcp(2,2)) , 2 ) ); //bz
};





//////////////////////// MAIN FUNCTION //////////////////////////////////////////////////
bool ikHandler::solveIK(const Eigen::VectorXd& _target){
    target = _target;
    status = true;

    // Numerical IK
    if (OptVarDim > 6 || useNumIK){
        solution.resize(OptVarDim,1);
        
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

        if (!optSuccess || f_val > 1e-6)
            status = false;

        for (int i=0; i < OptVarDim; i++)
            solution(i,0) = iterator[i];
        return status;
    }
    // Analytical IK
    else{
        solution.resize(OptVarDim,1);
        Eigen::MatrixXd sol_mat;
        ik_analytical::compute_IK(target,status,sol_mat);
        double config_dist = 10000000;
        for (int i=0; i<sol_mat.rows(); ++i){
            // Check the Limits
            if (sol_mat(i,0)>=robot->Joints_ll(0) && sol_mat(i,0)<=robot->Joints_ul(0) &&
                sol_mat(i,1)>=robot->Joints_ll(1) && sol_mat(i,1)<=robot->Joints_ul(1) &&
                sol_mat(i,2)>=robot->Joints_ll(2) && sol_mat(i,2)<=robot->Joints_ul(2) &&
                sol_mat(i,3)>=robot->Joints_ll(3) && sol_mat(i,3)<=robot->Joints_ul(3) &&
                sol_mat(i,4)>=robot->Joints_ll(4) && sol_mat(i,4)<=robot->Joints_ul(4) &&
                sol_mat(i,5)>=robot->Joints_ll(5) && sol_mat(i,5)<=robot->Joints_ul(5) ){
                // Add it to solution. Also check if this is closest to given config
                if ((sol_mat.row(i).transpose() - init_guess).norm() < config_dist)
                    solution.col(0) = sol_mat.row(i).transpose();
                solution.conservativeResize(OptVarDim,solution.cols()+1);
                solution.col(solution.cols()-1) = sol_mat.row(i).transpose();
            }
        }
        return status;
    }
}