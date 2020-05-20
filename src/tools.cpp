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
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // extracting state values
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[3];
  float vy = x_state[4];
  
  // repeating calculations for faster matrix calculation
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // addressing zero error issues
  if (c1 < 0.0001){
     std::cout << "Error: Division by zero" << std::cout;
     return Hj;
  }
  
  // Matrix Hj calculation based on Jacobian matrix equations
  Hj << px/c2,py/c2,0,0,
        -py/c1,px/c1,0,0,
        py*(vx*py - vy*px)/c3,px*(vy*px - vx*py)/c3,px/c2,py/c2;

  return Hj;
}
