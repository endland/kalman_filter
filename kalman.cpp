/**
 * Simple Linear Kalman-filter example
 *
 */

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A, // F
    const Eigen::MatrixXd& H, // H 
    const Eigen::MatrixXd& Q, // Q
    const Eigen::MatrixXd& R, // R
    const Eigen::MatrixXd& P) // P
  : A(A), H(H), Q(Q), R(R), P0(P),
    m(H.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)  // constructor member initializer lists
{
  I.setIdentity();
}



KalmanFilter::KalmanFilter() {}  // define default constructor

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;                                  // XPred = F*X
  P = A*P*A.transpose() + Q;                              // PPred = F*P*F' + Q
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();  // K = PPred*H'/S, S = C*P*C' + R, 
  x_hat_new += K * (y - H*x_hat_new);                     // XEst = XPred + K*y, y=z-H*XPred
  P = (I - K*H)*P;                                        // PEst = (I - K*H)*PPred, P = PEst
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}
