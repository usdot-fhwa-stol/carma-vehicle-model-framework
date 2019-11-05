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
#include "passenger_car_dynamic_model/TwoStepPID.h"


/**
 * Tests the TwoStepPID class
 */ 
TEST(TwoStepPID, testPID)
{

  // Test P controller
  TwoStepPID P(1.0, 0.0, 0.0);
  
  P.setSetpoint(1.0);

  ASSERT_NEAR(1.0, P.computeOutput(0.0, 1.0), 0.000000001);
    
  P.updateMemory(0.0, 1.0);

  ASSERT_NEAR(1.0, P.computeOutput(0.0, 1.0), 0.000000001); // No change as only P controller

  // Test I Controller
  TwoStepPID I(0.0, 0.1, 0.0);

  I.setSetpoint(1.0);

  ASSERT_NEAR(0.0, I.computeOutput(0.0, 0.0), 0.000000001); // First loop returns 0 as no dt
  ASSERT_NEAR(0.1, I.computeOutput(0.0, 1.0), 0.000000001); // 1s timestep
  ASSERT_NEAR(0.1, I.computeOutput(0.0, 1.0), 0.000000001); // Check that memory is not retained

  I.updateMemory(0.0, 1.0);
  
  ASSERT_NEAR(0.0, I.computeOutput(0.0, 1.0), 0.000000001); // If time does not change then dt = 0
  ASSERT_NEAR(0.15, I.computeOutput(0.5, 2.0), 0.000000001); // Check integrator impacts result

  I.setOutputMin(0.2);

  ASSERT_NEAR(0.2, I.computeOutput(0.5, 2.0), 0.000000001); // Check output min override

  I.setOutputMin(-1000.0);
  I.setOutputMax(0.1);

  ASSERT_NEAR(0.1, I.computeOutput(0.5, 2.0), 0.000000001); // Check output max override

  I.setOutputMax(1000.0);

  I.updateMemory(0.0, 2.0);

  ASSERT_NEAR(0.25, I.computeOutput(0.5, 3.0), 0.000000001); // Check integrator soft reset

  I.setOutputMax(0.1);

  I.updateMemory(0.5, 3.0);

  // Evaluate D
  TwoStepPID D(0.0, 0.0, 1.0);

  D.setSetpoint(1.0);

  D.updateMemory(0.0, 1.0);

  ASSERT_NEAR(-0.5, D.computeOutput(0.5, 2.0), 0.000000001); // Check prev error impacts result

  // Evaluate reversing output check
  TwoStepPID Drev(0.1, 0.0, 1.0);

  Drev.setSetpoint(1.0);

  Drev.updateMemory(0.0, 1.0);

  ASSERT_NEAR(0.0, Drev.computeOutput(0.5, 2.0), 0.000000001); // Check prev error impacts result

  // Test Output Max
  TwoStepPID PMax(10, 0.0, 0.0);
  PMax.setOutputMax(1);
  PMax.setOutputMin(-1);
  PMax.setSetpoint(100.0);
  PMax.updateMemory(0.0, 0.0);
  ASSERT_NEAR(1.0, PMax.computeOutput(0.0, 2.0), 0.000000001); // Check prev error impacts result
  PMax.setSetpoint(-100.0);
  ASSERT_NEAR(-1.0, PMax.computeOutput(0.0, 2.0), 0.000000001); // Check prev error impacts result
}
