#include <passenger_car_dynamic_model/TwoStepPID.h>
#include <iostream>

TwoStepPID::TwoStepPID(double kP, double kI, double kD) : kP_(kP), kI_(kI), kD_(kD) {}



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
  const double error = setpoint_ - state;
  const double dt = time - prev_time_;
  double output = kP_ * error;

  if (dt < 0.00000001) { // For first loop we only want to use P value
    return output;
  }

   // Update Integrator
  double integrator = integrator_ + (error * dt); // Local integrator
  output += kI_ * integrator;

  output += kD_ * ((error - prev_error_) / dt);

  if(output >  output_max_) {
    return  output_max_;
  } else if(output < output_min_) {
    return output_min_;
  } else {
    return output;
  }
}

void TwoStepPID::updateMemory(double state, double time) {
  // Update PID states
  const double error = setpoint_ - state;
  prev_error_ = error; // Update last error value
  const double dt = time - prev_time_;

  //std::cerr << " Update Memory State: " << state << " Time: " << time << " Setpoint: " << setpoint_ << " Error: " << error << " dt: " << dt << std::endl; 


  prev_time_ = time; // Update last time value

  // Update integrator value
  integrator_ += (error * dt);

  const double PD = kP_ * error + kD_ * ((error - prev_error_) / dt);
  const double I = kI_ * integrator_;
  const double prevOutput = PD + I;

  //std::cerr << " Output " << prevOutput << std::endl; 

  // Check for integrator windup causing output overshoot and reset integrator to result giving min/max output if overshoot is caused by it
  if(prevOutput > output_max_ && PD < output_max_) {
    integrator_ -= (prevOutput - output_max_) / kI_;
  } else if(prevOutput < output_min_ && PD > output_min_) {
    integrator_ -= (prevOutput - output_min_) / kI_;
  }
}
