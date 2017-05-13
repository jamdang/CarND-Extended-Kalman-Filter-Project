#include "kalman_filter.h"
#include <iostream>     /* debug */
//#include <math.h>       /* atan2 */

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  // debug 
  //std::cout<<"Entered the Predict()..."<<std::endl;
  
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  /* //debug
  std::cout << "Enter Update()" << std::endl;
  std::cout << "R_: " << R_ << std::endl;  */
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //debug
  //std::cout << "K:  " << K <<std::endl;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  /* //debug
  std::cout << "Enter UpdateEKF()" << std::endl;
  std::cout << "R_: " << R_ << std::endl; 
  std::cout << "Hj: " << H_ << std::endl;  */
  
  // calc z_pred, i.e., predicted measurement
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho     = sqrt(px*px + py*py);
  float phi;
  if (abs(px) < 1e-3){
	  if (py > 1e-3){
		  phi = PI/2;
	  }else if(py < 1e-3){
		  phi = -PI/2;
	  }else{
		  phi = 0.0f;
	  }	  
  }else{
	  phi = atan2(py,px);
  }

  float rho_dot;
  if (rho < 1e-3){
	  rho_dot = 0.0f;
  }else{
	  rho_dot = (px*vx + py*vy)/rho;
  }
  
  /* // debug   
  VectorXd z_approx = H_ * x_; 
  std::cout << "estimated rho: " << rho << std::endl;
  std::cout << "true rho     : " << z(0) << std::endl;
  std::cout << "estimated phi: " << phi << std::endl;
  std::cout << "true phi:      " << z(1) << std::endl;
  std::cout << "estimated rho_dot: " << rho_dot<< std::endl;
  std::cout << "true rho_dot:      " << z(2) << std::endl; */
  
  
  VectorXd z_pred = VectorXd(3);    
  z_pred << rho, phi, rho_dot;
  // calc innovation
  VectorXd y = z - z_pred; 
  // modify delta phi  
  if ( 2*PI - abs(y(1)) < abs(y(1))){	  
	  if (y(1) > 0) 
		  y(1) -= 2*PI;
	  else
		  y(1) += 2*PI;	  
  }   
   
  // calc Kalman 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
/*   // debug     
  std::cout << "y:  " << y << std::endl;
  std::cout << "K:  " << K << std::endl; */
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;  
  
}
