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
 * @class TwoStepPID
 * @brief Class which implements a basic pid controller where the value calculation and memory update can be handled seperately
 * 
 * By seperating the output calculation and memory update steps this PID controller can be used in systems which check multiple options before commiting to an action
 * However, as the memory step requires a recalculation of the error term there is a small overhead incurred for the added flexability
 * 
 * 
 */
class TwoStepPID
{
  private:

    const double kP_ = 1.0;
    const double kI_ = 0.0;
    const double kD_ = 0.0;

    double setpoint_ = 0.0;
    double integrator_ = 0;
    double prev_error_ = 0;
    double prev_time_ = 0;
    double output_max_ = std::numeric_limits<double>::max();
    double output_min_ = std::numeric_limits<double>::lowest();


  public:

    TwoStepPID(double kP, double kI, double kD);

    void setSetpoint(double setpoint);

    void setOutputMax(double output_max);

    void setOutputMin(double output_min);

    double computeOutput(double state, double time);

    void updateMemory(double state, double time);

  private:
    // TODO remove
    //double capIntegrator(double integrator);

};
