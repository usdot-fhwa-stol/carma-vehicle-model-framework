#include <passenger_car_dynamic_model/TwoStepPID.h>
#include <iostream>
#include <math.h>
#include <fstream> // TODO remove
#include <iomanip>

TwoStepPID::TwoStepPID(double kP, double kI, double kD) : kP_(kP), kI_(kI), kD_(kD) {}


void TwoStepPID::setConstants(double kP, double kI, double kD) {
  kP_ = kP;
  kI_ = kI;
  kD_ = kD;
}

void TwoStepPID::setSetpoint(double setpoint) {
  setpoint_ = setpoint;
}

void TwoStepPID::setOutputMax(double output_max) {
  output_max_ = output_max;
}

void TwoStepPID::setOutputMin(double output_min) {
  output_min_ = output_min;
}

double TwoStepPID::computeOutput(double state, double time) {
  const double error = setpoint_ - state; // m/s - m/s = m/s
  const double dt = time - prev_time_;
  double output = kP_ * error; // C * m/s = m/s

  if (dt < 0.00000001) { // For first loop we only want to use P value
    return output;
  }

  
  if (fabs(output) > 0.1) {
  //std::cerr << "Output " << std::min(std::max(output, output_min_), output_max_) << std::endl;
  }

  //double final_output = (output - prev_output_) / dt;
  double final_output = (error - prev_error_) / dt;

  return std::min(std::max(final_output, output_min_), output_max_);
}

// Updates prev_error prev_output and prev_time fields
void TwoStepPID::updateMemory(double state, double time) {

  // Update PID states
  const double error = setpoint_ - state;
  const double dt = time - prev_time_;

  const double P = kP_ * error;

  prev_output_ = P;

  prev_error_ = error; // Update last error value
  prev_time_ = time; // Update last time value
}
