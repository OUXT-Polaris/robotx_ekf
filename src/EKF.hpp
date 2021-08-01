#include <Eigen/Dense>

#pragma once

class EKF {

public:

  EKF(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& M,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P0
  );

  EKF(); 
  void init(const Eigen::MatrixXd& P0,const Eigen::VectorXd& x0);
  void update(double dt, const Eigen::VectorXd& y);


private:

  // Matrices for computation
  Eigen::MatrixXd A, B, C, Q, R, P, M, K, P0;

  // System dimensions
  int m, n;

  double dt, t;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x, y, u, x0;

};