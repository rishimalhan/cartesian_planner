#include <gurobi_c++.h>

class jacQP{
public:
  GRBEnv* env = new GRBEnv();
  GRBModel model = GRBModel(*env);
  int dimension;
  GRBQuadExpr obj;
  jacQP(int _dof){
    dimension = _dof;
    // Options
    model.set("OutputFlag","0");
    model.set("LogToConsole","0");
  };

  Eigen::MatrixXd solveQP(const Eigen::MatrixXd& init_guess, const Eigen::MatrixXd& Jac, 
      const Eigen::MatrixXd& x, const Eigen::MatrixXd& jt_lb, const Eigen::MatrixXd& jt_ub){
    Eigen::MatrixXd jtj = 2*(Jac.transpose() * Jac); // Q matrix nxn
    Eigen::VectorXd xtj = -2*(Jac.transpose() * x.col(0)).col(0); // c column vector nx1
    double lb[dimension];
    double ub[dimension];
    double  objval, sol[dimension];
    Eigen::MatrixXd jt_config(dimension,1);

    for (int i = 0; i < dimension; ++i){
      lb[i] = jt_lb(i,0) - init_guess(i,0);
      ub[i] = jt_ub(i,0) - init_guess(i,0);
    }

    /* Add variables to the model */
    GRBVar* vars = model.addVars(lb, ub, NULL, NULL, NULL, dimension);

    GRBQuadExpr obj = x.norm();

    for (int i = 0; i < dimension; ++i){
      obj += xtj(i)*vars[i];
      for (int j = 0; j < dimension; ++j)
        if (jtj(i,j) != 0)
          obj += jtj(i,j)*vars[i]*vars[j];
    }

    model.setObjective(obj);
    model.optimize();
    // model.set("TuneCriterion","2");
    // model.tune();

    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
      // std::cout<< "Success!!\n";
      // double *objvalP = model.get(GRB_DoubleAttr_ObjVal);
      for (int i = 0; i < dimension; ++i)
        jt_config(i,0) = vars[i].get(GRB_DoubleAttr_X);
    }
    else
      // std::cout<< "Failure!!\n";

    delete[] vars;

    // std::cout<< jt_config << "\n";
    return jt_config;
    // } catch(GRBException e) {
    //   cout << "Error code = " << e.getErrorCode() << endl;
    //   cout << e.getMessage() << endl;
    // } catch(...) {
    //   cout << "Exception during optimization" << endl;
    // }
  }
};