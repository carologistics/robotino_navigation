// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/mimo.hpp"

MIMO_PID::MIMO_PID(const Eigen::MatrixXd Kp, const Eigen::MatrixXd Ki,
                   const Eigen::MatrixXd Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd) {
  // Initialize as column vectors to match error matrix dimensions (3x1)
  this->integral = Eigen::MatrixXd::Zero(Kp.cols(), 1);
  this->derivative = Eigen::MatrixXd::Zero(Kp.cols(), 1);
  this->error_prev = Eigen::MatrixXd::Zero(Kp.cols(), 1);
}

MIMO_PID::~MIMO_PID() {}

MIMO_PID::MIMO_PID() {
  this->integral = Eigen::MatrixXd::Zero(3, 1);
  this->derivative = Eigen::MatrixXd::Zero(3, 1);
  this->error_prev = Eigen::MatrixXd::Zero(3, 1);
}

void MIMO_PID::set_Kp(const Eigen::MatrixXd Kp) { 
  this->Kp = Kp; 
  // Reinitialize integral, derivative, error_prev to match new dimensions
  if (this->integral.rows() != Kp.cols() || this->integral.cols() != 1) {
    this->integral = Eigen::MatrixXd::Zero(Kp.cols(), 1);
    this->derivative = Eigen::MatrixXd::Zero(Kp.cols(), 1);
    this->error_prev = Eigen::MatrixXd::Zero(Kp.cols(), 1);
  }
}

void MIMO_PID::set_Ki(const Eigen::MatrixXd Ki) { 
  this->Ki = Ki; 
  // Reinitialize if dimensions don't match
  if (this->integral.rows() != Ki.cols() || this->integral.cols() != 1) {
    this->integral = Eigen::MatrixXd::Zero(Ki.cols(), 1);
    this->derivative = Eigen::MatrixXd::Zero(Ki.cols(), 1);
    this->error_prev = Eigen::MatrixXd::Zero(Ki.cols(), 1);
  }
}

void MIMO_PID::set_Kd(const Eigen::MatrixXd Kd) { 
  this->Kd = Kd; 
  // Reinitialize if dimensions don't match
  if (this->integral.rows() != Kd.cols() || this->integral.cols() != 1) {
    this->integral = Eigen::MatrixXd::Zero(Kd.cols(), 1);
    this->derivative = Eigen::MatrixXd::Zero(Kd.cols(), 1);
    this->error_prev = Eigen::MatrixXd::Zero(Kd.cols(), 1);
  }
}

Eigen::MatrixXd MIMO_PID::compute(const Eigen::MatrixXd error,
                                  const double dt) {
  this->integral += error * dt;
  this->derivative = (error - this->error_prev) / dt;
  this->error_prev = error;
  return this->Kp * error + this->Ki * this->integral +
         this->Kd * this->derivative;
}