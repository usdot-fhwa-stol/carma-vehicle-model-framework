#include <passenger_car_dynamic_model/TwoStepLagModel.h>
#include <iostream>
#include <math.h>

TwoStepLagModel::TwoStepLagModel(double step_change_lag_time, double crawl_speed) 
  : step_change_lag_time_(step_change_lag_time), crawl_speed_(crawl_speed) {}

    // const double low_speed_lag_time_;
    // const double crawl_speed_;

    // bool prev_setpoint_set = false;
    // bool lag_complete_;
    // double setpoint_;
    // double prev_setpoint_;
    // double lag_start_time_;


void TwoStepLagModel::setSetpoint(double setpoint) {
  if (setpoint_ != prev_setpoint_) {
    prev_setpoint_ = setpoint_; // Store prev setpoint
  }
  setpoint_ = setpoint; 
}

double TwoStepLagModel::computeOutput(double state, double zero_lag_response, double time) {
  std::cerr <<  "Output: " << "state " << state << " zero_lag_response " << zero_lag_response << " time " << time << std::endl;
  std::cerr << "lag_start_time_ " << lag_start_time_ << " step_change_lag_time_ " << step_change_lag_time_ << " lag_complete_ " << lag_complete_ << std::endl << std::endl;

  // If lag is still ocurring return current state
  if (time - lag_start_time_ < step_change_lag_time_ && !lag_complete_) {
    return state;
  }

  // If lag is no longer occuring return zero-lag response
  return zero_lag_response;
}

void TwoStepLagModel::updateMemory(double state, double zero_lag_response, double time) {
  // if (setpoint_ < crawl_speed_) {
  //     lag_complete_ = true;
  //   }

  // TODO
  /// HERE
  // HERE
  // HERE
  // IT is not clear if there is any lag time for small speed changes. However there does seem to be one for large step changes
  // For changes in speed less then 5mph assume no lag time. 
  // But for speed changes above 2.2m/s assume a constant value of 0.17s based on the results
  // TODO make 2.2 a parameter
  if (fabs(setpoint_ - prev_setpoint_) > 2.2 && lag_complete_) {
    std::cerr<< "Starting lag" << std::endl;
    lag_start_time_ = time;
    lag_complete_ = false;
  }

  if (time - lag_start_time_ > step_change_lag_time_ && !lag_complete_) {
    lag_complete_ = true;
  }
}

// void TwoStepLagModel::updateMemory(double state, double zero_lag_response, double time) {
//   // if (setpoint_ < crawl_speed_) {
//   //     lag_complete_ = true;
//   //   }


//   // If the serpoint will cause the vehicle to accelerate 
//   // and the previous setpoint was above the crawling speed and the vehicle is not currently lagging
//   if(state < crawl_speed_ && zero_lag_response > 0 && lag_complete_) {
//     std::cerr<< "Starting lag" << std::endl;
//     lag_start_time_ = time;
//     lag_complete_ = false;
//   } else if (zero_lag_response < 0) {
//     lag_complete_ = true;
//   }
//   // if (setpoint_ > state && state < crawl_speed_ && lag_complete_ && setpoint_ != prev_setpoint_) {
//   //   // Start of lag period
//   //   std::cerr<< "Starting lag" << std::endl;
//   //   lag_start_time_ = time;
//   //   lag_complete_ = false;
//   // }
//  ///
//  /// NOTE: It is possible that the Lag is due entirely to gear shifting. We need to confirm
//  /// NOTE: It seems lag time is actually a consistent 0.25 seconds. Unclear if this changes with speed. Suggest basing lag time on setpoint changes and reduces as difference between setpoints drops
//  /// Match table [(10, 0.21) (-5, 0.14) (10, 0.12) (10, .16) (-5, 0.2)] That gives a near constant response lag of 0.17
//  ///
//   // Do not allow for lag to end unless the setpoint is above the crawl_speed_
//   if (setpoint_ < crawl_speed_) {
//     lag_start_time_ = time;
//   }
//   if (time - lag_start_time_ > low_speed_lag_time_ && !lag_complete_) {
//     lag_complete_ = true;
//   }
// }
