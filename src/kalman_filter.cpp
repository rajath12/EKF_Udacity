#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  // for lidar measurements

  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  MatrixXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_ * Ht * S_inv;

  // measurement update
  x_ = x_ + K * y;
  P_ = (I - K*H_) *P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // for radar measurements

  // converting cartesian to polar coordinates
  // calculate rho, phi and rho_dot
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1),x_(0));
  float rho_dot; // zero case for rho needs to be dealt with
  if (fabs(rho) < 0.0001)
  {
    float rho_dot = 0;
  } else {
    float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }

  MatrixXd hx(3); // polar coordinate predictions for calculating error y
  hx << rho, phi, rho_dot;
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  MatrixXd y = z - hx;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_ * Ht * S_inv;

  // measurement update
  x_ = x_ + K * y;
  P_ = (I - K*H_) *P_;
}
