#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  
  // EKF Lecture 8:
  VectorXd u(4);
  u << 0, 0, 0, 0; // don't need u, this doesn't update anywhere later
  
  // KF Prediction step
  x_ = F_ * x_ + u;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  // EKF Lecture 7:
  
  // KF Measurement update step
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  // From EKF lecture 21:
  // Convert state to polar coordinates
  // Use H(x) instead of H, and calculate S, K, and P
  // y becomes z - h(x') for radar
  // h(x') is [rho; phi; rho_dot]
  // rho is distance to object
  // phi is angle to object
  // rho_dot is the projected velocity in phi direction
  
  // recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // convert to polar coordinates using equations from lecture 21
  float rho = sqrt((px*px) + (py*py));
  float phi = atan2(py, px);
  float rho_dot;
  
  const float pi = 3.14159265358979323846;
  
  if (fabs(rho) <= 0.01) {
    rho_dot = 0;
  } else {
    rho_dot = (px*vx + py*vy)/rho;
  }
  
  // EKF Measurement Update for Radar
  VectorXd hx(3);
  hx << rho, phi, rho_dot;
  
  VectorXd y = z - hx;  
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  // normalize phi to keep in range [-pi,pi]
  while (y(1) < -pi) {
    y(1) += 2*pi;
  }
  while (y(1) > pi) {
    y(1) -= 2*pi;
  }
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
