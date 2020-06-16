#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

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
  I_ =  MatrixXd::Identity(4, 4);
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
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    // new state
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_) * P_;   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
	// radar, z is in the format of 
	// h(x) to compute current position from cartesian coordinates to polar coordinates to compute error
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];
	
	float rho = sqrt(px*px + py*py);
	if (fabs(rho) < 0.0001) {
		cout << "UpdateEKF () - Error - Division by Zero" << endl;
		return;
	}

	float phi = std::atan2(py,px);
	cout << "phi:" << phi  << endl;
	float rho_dot = (px*vx + py*vy)/rho;
	VectorXd z_polar(3);
	z_polar << rho, phi, rho_dot;
	
    VectorXd y = z - z_polar; //3x1
    MatrixXd Ht = H_.transpose(); //Hj is 3x4, so Ht is 4x3 
    MatrixXd S = H_ * P_ * Ht + R_; //3x3
    MatrixXd Si = S.inverse(); //3x3
    MatrixXd K =  P_ * Ht * Si; //4x3

    // new state
    x_ = x_ + (K * y); //4x1
    P_ = (I_ - K * H_) * P_; //4x4    
}
