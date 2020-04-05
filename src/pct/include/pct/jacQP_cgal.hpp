// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function is an implementation of a quadratic program for finding an inverse kinematics solution for a given target,
//  previous configuration and constraints imposed.
// OBJECTIVE FUNCTION TO MIN: 1/2*q'J'Jq - x'Jq where x is difference between waypoints wi and w(i-1). q is the 
// decision variable which is change in configuration required from initial configuration. 



#include <iostream>
#include <Eigen/Eigen>
#include <pct/cholesky_decomposition.hpp>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
// choose exact integral type
#ifdef CGAL_USE_GMP
// #include <CGAL/Gmpz.h>
// typedef CGAL::Gmpz ET;
// #else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif
// program and solution types
typedef CGAL::Quadratic_program_from_iterators
<double**,                                                // for A
    double*,                                                 // for b
    CGAL::Const_oneset_iterator<CGAL::Comparison_result>, // for r
    bool*,                                                // for fl
    double*,                                                 // for l
    bool*,                                                // for fu
    double*,                                                 // for u
    double**,                                                // for D
    double*>                                                 // for c 
Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

Eigen::MatrixXd solveQP(const Eigen::MatrixXd& init_guess, const Eigen::MatrixXd& Jac, 
    const Eigen::MatrixXd& x, const Eigen::MatrixXd& jt_lb, const Eigen::MatrixXd& jt_ub){
    Eigen::MatrixXd jtj = 2*(Jac.transpose() * Jac);
    Eigen::MatrixXd xtj = -2*(Jac.transpose() * x.col(0));

    // cholesky factorizer(init_guess.rows());
    // factorizer.ldl(jtj);
    // Eigen::MatrixXd L = factorizer.getL();
    // Eigen::MatrixXd Diag = factorizer.getD();  

    // for (int k = 0; k < Diag.rows(); k++)  { if(Diag(k,k)<=0) { Diag(k,k)=0; } }
    // jtj = L*Diag*L.transpose();

    double*  A[] = {};
    double   b[] = {};
    CGAL::Const_oneset_iterator<CGAL::Comparison_result> 
          r(    CGAL::SMALLER);
    bool fl[] = {true, true};
    double   l[init_guess.rows()];
    bool fu[] = {true, true};
    double   u[init_guess.rows()];
    double**  D; // Row wise elements R1,R2,...
    D = new double*[init_guess.rows()];
    double   c[init_guess.rows()];
    double  c0   = x.col(0).norm();
    for (int i=0; i<init_guess.rows(); ++i){
      l[i] = jt_lb(i,0) - init_guess(i,0);
      u[i] = jt_ub(i,0) - init_guess(i,0);
      c[i] = xtj(i,0);
      D[i] = new double[init_guess.rows()];
      for (int j=0; j<init_guess.rows(); ++j){
          D[i][j] = jtj(i,j);
      }
    }
    CGAL::Quadratic_program_options options;

    // now construct the quadratic program; the first two parameters are
    // the number of variables and the number of constraints (rows of A)
    Program qp (init_guess.rows(), 0, A, b, r, fl, l, fu, u, D, c, c0);
    // solve the program, using ET as the exact type
    Solution s = CGAL::solve_quadratic_program(qp, ET(),options);
    // output solution
    std::cout << s;

    // if (!s.solves_quadratic_program(qp))
    //   std::cout<< "FUCCCCCKKKK\n";

    delete[] D;

    Eigen::VectorXd sol(1,1);
    sol << 1;
    return sol;
}
