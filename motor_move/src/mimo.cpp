// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/mimo.hpp"

MIMO_PID::MIMO_PID(const Eigen::MatrixXd Kp, const Eigen::MatrixXd Ki,
                   const Eigen::MatrixXd Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd) {
  this->integral = Eigen::MatrixXd::Zero(1, Kp.rows());
  this->derivative = Eigen::MatrixXd::Zero(1, Kp.rows());
  this->error_prev = Eigen::MatrixXd::Zero(1, Kp.rows());
}

MIMO_PID::~MIMO_PID() {}

MIMO_PID::MIMO_PID() {
  this->integral = Eigen::MatrixXd::Zero(3, 1);
  this->derivative = Eigen::MatrixXd::Zero(3, 1);
  this->error_prev = Eigen::MatrixXd::Zero(3, 1);
}

void MIMO_PID::set_Kp(const Eigen::MatrixXd Kp) { this->Kp = Kp; }

void MIMO_PID::set_Ki(const Eigen::MatrixXd Ki) { this->Ki = Ki; }

void MIMO_PID::set_Kd(const Eigen::MatrixXd Kd) { this->Kd = Kd; }

Eigen::MatrixXd MIMO_PID::compute(const Eigen::MatrixXd error,
                                  const double dt) {
  this->integral += error * dt;
  this->derivative = (error - this->error_prev) / dt;
  this->error_prev = error;
  return this->Kp * error + this->Ki * this->integral +
         this->Kd * this->derivative;
}
