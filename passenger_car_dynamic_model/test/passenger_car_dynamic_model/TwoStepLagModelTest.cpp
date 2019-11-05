/*
 * Copyright (C) 2019 LEIDOS.
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "passenger_car_dynamic_model/TwoStepLagModel.h"


/**
 * Tests the TwoStepLagModel class
 */ 
TEST(TwoStepLagModel, test_lag)
{
  // const double lag_time = 0.5; // s
  // const double crawl_speed = 2.25; // m/s
  // TwoStepLagModel model(lag_time, crawl_speed);

  // model.setSetpoint(0);
  // ASSERT_EQ(0.0, model.computeOutput(0,0,0));
  // ASSERT_EQ(0.0, model.computeOutput(0,0,0.25));

  // // Update memory with no new setpoint and as initial point
  // model.updateMemory(0.0, 0.0, 0.0);
  // ASSERT_EQ(0.0, model.computeOutput(0,0,0));
  // ASSERT_EQ(0.0, model.computeOutput(0,0,0.25));

  // // Update setpoint
  // model.setSetpoint(5);
  // // Update memory to 0.5s Start of lag period
  // std::cerr << "Expected Lag Start" << std::endl;
  // model.updateMemory(0.0, 0.0, 0.5);
  // ASSERT_EQ(0.0, model.computeOutput(0,0,0.5));

  // // 0.75s  In lag
  // model.updateMemory(0.0, 0.1, 0.75);
  // ASSERT_EQ(0.0, model.computeOutput(0,0.1,0.75));

  // // 1.0s End of lag
  // model.updateMemory(0.0, 0.2, 1.0);
  // ASSERT_EQ(0.2, model.computeOutput(0.0, 0.2, 1.0));

  // // 1.25s Lag still over
  // model.updateMemory(0.1, 0.3, 1.25);
  // ASSERT_EQ(0.3, model.computeOutput(0.1, 0.3, 1.25));

  // // 2.5s End of lag response
  // model.updateMemory(0.0, 0.4, 2.5);
  // ASSERT_EQ(0.4, model.computeOutput(0.0, 0.4, 2.5));

  // // 3.0s
  // model.updateMemory(0.2, 0.5, 3.0);
  // ASSERT_EQ(0.5, model.computeOutput(0.2, 0.5, 3.0));
  
  // // 3.5s
  // model.updateMemory(0.4, 0.5, 3.5);
  // ASSERT_EQ(0.5, model.computeOutput(0.4, 0.5, 3.5));

  // // 4.0s Jump current speed above crawl speed
  // model.updateMemory(5.0, 0.1, 4.0);
  // ASSERT_EQ(0.1, model.computeOutput(5.0, 0.1, 4.0));

  // // 4.5s Set new setpoint. THere should be no lag
  // // Update setpoint
  // model.setSetpoint(6);
  // model.updateMemory(5.0, 0.2, 4.5);
  // ASSERT_EQ(0.2, model.computeOutput(5.0, 0.2, 4.5));

  // // 5s Set new setpoint. There should be no lag on a deceleration
  // // Update setpoint
  // model.setSetpoint(4);
  // model.updateMemory(5.1, -0.3, 5.0);
  // ASSERT_EQ(-0.3, model.computeOutput(5.1, -0.3, 5.0));

  // // 5.5s Still no lag
  // model.updateMemory(4.8, -0.3, 5.5);
  // ASSERT_EQ(-0.3, model.computeOutput(4.8, -0.3, 5.5));

  // // Reverse directions
  // // Update setpoint
  // model.setSetpoint(7);
  // // 6s Still no lag
  // model.updateMemory(4.7, 0.4, 6.0);
  // ASSERT_EQ(0.4, model.computeOutput(4.7, 0.4, 6.0));

  // // Reverse directions and head to 0
  // // Update setpoint
  // model.setSetpoint(0.0);
  // // 6.5s Still no lag
  // model.updateMemory(4.7, -0.5, 6.5);
  // ASSERT_EQ(-0.5, model.computeOutput(4.7, -0.5, 6.5));

  // // 7.0s Still no lag
  // model.updateMemory(4.0, -0.5, 7.0);
  // ASSERT_EQ(-0.5, model.computeOutput(4.0, -0.5, 7.0));

  // // 7.5s Jump speed to 1 to reset lag computation
  // model.updateMemory(1.0, -0.5, 7.5);
  // ASSERT_EQ(-0.5, model.computeOutput(1.0, -0.5, 7.5));

  // // 8.0s Still no lag as still decelerating
  // model.updateMemory(0.5, -0.1, 8.0);
  // ASSERT_EQ(-0.1, model.computeOutput(0.5, -0.1, 8.0));

  // // 8.5s Acceleration should enforce lag
  // // Update setpoint
  // model.setSetpoint(3.0);
  // model.updateMemory(0.5, 2.0, 8.5);
  // ASSERT_EQ(0.5, model.computeOutput(0.5, 2.0, 8.5));

  // // 9.0s Still lagging
  // model.updateMemory(0.6, 2.0, 9.0);
  // ASSERT_EQ(0.5, model.computeOutput(0.6, 2.0, 9.0));

  // // 10.0s Jump to end lag
  // model.updateMemory(0.7, 2.0, 10.0);
  // ASSERT_EQ(2.0, model.computeOutput(0.7, 2.0, 10.0));
}
