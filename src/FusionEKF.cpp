#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
   
  H_laser_ << 1,0,0,0,
              0,1,0,0;
			  
  Qv_ = MatrixXd(2, 2);
  Qv_ << 9,0,
         0,9;
		 
//  ekf_.Init(VectorXd(4), MatrixXd(4,4), MatrixXd(4,4),
//			MatrixXd(3,4), MatrixXd(3,3), MatrixXd(4,4));		 
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
//  cout << measurement_pack.raw_measurements_[0] << measurement_pack.raw_measurements_[1]  << endl;
//    cout << measurement_pack.raw_measurements_(0) << measurement_pack.raw_measurements_(1)  << endl;
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
	  float rho = measurement_pack.raw_measurements_[0];
	  float phi = measurement_pack.raw_measurements_[1];
	  float rho_dot = measurement_pack.raw_measurements_[2];
	  
	  ekf_.x_[0] = rho*cos(phi);
	  ekf_.x_[1] = rho*sin(phi);	
	  ekf_.x_[2] = rho_dot*cos(phi);
	  ekf_.x_[3] = rho_dot*sin(phi);	  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	  ekf_.x_[0] = measurement_pack.raw_measurements_[0];
	  ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }
	
	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_ << 1, 0, 0, 0,
			   0, 1, 0, 0,
			   0, 0, 1000, 0,
			   0, 0, 0, 1000;
	
	ekf_.I_ = MatrixXd::Identity(4, 4);
	previous_timestamp_ = measurement_pack.timestamp_;
	
    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   
  // update F_ matrix 
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0; //convert us to second
  ekf_.F_ = MatrixXd(4,4);  
  ekf_.F_ << 1,0,dt,0,
			 0,1,0,dt,
			 0,0,1,0,
			 0,0,0,1;
			 
  // update Q_ matrix			 
  MatrixXd G(4,2); //G matrix for Q computation. Q = G*Qv*G.transpose
  G << dt*dt/2,0,
	   0,dt*dt/2,
	   dt,0,
	   0,dt;
  ekf_.Q_ = G*Qv_*G.transpose();
	
  // call KF to predict	
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
	Hj_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.H_ = Hj_; //3x4 Jacobian matrix for radar sensor (function of x_state)
	ekf_.R_ = R_radar_; //3x3 laser noise matrix
	ekf_.UpdateEKF(measurement_pack.raw_measurements_); //linear KF	
	
  } else {
    // TODO: Laser updates
	ekf_.H_ = H_laser_; //2x4 output matrix
	ekf_.R_ = R_laser_; //2x2 laser noise matrix
	ekf_.Update(measurement_pack.raw_measurements_); //linear KF
  }

  previous_timestamp_ = measurement_pack.timestamp_; 
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
