#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// RMSE function tested and okayed
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // initializing rmse vector
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // checking valid inputs to the function
  if ((estimations.size() != ground_truth.size()) || (estimations.size() == 0) )
  {
     std::cout << "Invalid estimaions and ground truths" << std::endl;
     return rmse;
  }
  // add up values of squared errors for all data points
  for (unsigned int i = 0; i < estimations.size(); ++i){ //uning unsigned int as negative values are not needed
     VectorXd residual = estimations[i] - ground_truth[i];
     residual = residual.array()*residual.array();
     rmse += residual;
  }

  rmse = rmse/estimations.size(); // normalizing
  rmse = rmse.array().sqrt(); // final square root calculation

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
