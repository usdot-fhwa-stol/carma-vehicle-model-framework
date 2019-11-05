#pragma once
/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <limits>

 
/**
 * @class TwoStepLagModel
 * @brief Class which models the response lag of the vehicle drive train
 * 
 * By seperating the output calculation and memory update steps, this lag model can be used in systems which check multiple options before commiting to an action
 * However, as the memory step requires a recalculation there is a small overhead incurred for the added flexability
 * 
 * 
 */
class TwoStepLagModel
{
  private:

    const double step_change_lag_time_;
    const double crawl_speed_;

    bool prev_setpoint_set = false;
    bool lag_complete_ = true;
    double setpoint_ = 0;
    double prev_setpoint_ = 0;
    double lag_start_time_ = 0;

  public:

    TwoStepLagModel(double step_change_lag_time, double crawl_speed);

    void setSetpoint(double setpoint);

    // void setOutputMax(double output_max);

    // void setOutputMin(double output_min);

    double computeOutput(double state, double zero_lag_response, double time);

    void updateMemory(double state, double zero_lag_response, double time);

  private:

};


// What is needed
// trackable lag complete flag (only update after integration)
// trackable setpoint
// trackable prevSetpoint (prevSetpoint should only be updated after integration)
// trackable lag start time (only update after integration?)
// double handleLag(double pid_result) {

//   if (setpoint < 2.23) {
//     lagComplete = true;
//   }
//   // TODO create variable for crawling speed
//   // lagComplete starts as true
//   // How to set prevSetpoint?
//   if (currentSpeed < 2.23) {
//     if (setpoint > currentSpeed && prevSetpoint < 2.23 && lagComplete) {
//       // Start of lag period
//       lagTimeStart = currentTime;
//       lagComplete = false;
//       return 0;
//     }
//   }

//   if (currentTime - lagStartTime > LAG_TIME && !lagComplete) {
//     lagComplete = true;
//   }

//   return pid_result
// }