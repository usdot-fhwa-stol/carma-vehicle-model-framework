/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <memory>
#include <gtest/gtest.h>
#include <gmock/gmock.h> 
#include "lib_vehicle_model/VehicleState.h"
#include "../src/lib_vehicle_model/ConstraintChecker.h"
#include "lib_vehicle_model/ParameterServer.h"

/**
 * This file unit tests the ConstraintChecker checker class
 */ 

using namespace lib_vehicle_model;
using ::testing::A;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::Unused;

class MockParamServer : public ParameterServer {
  public:
    MOCK_METHOD2(getParam, bool(const std::string& param_key, std::string& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, double& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, float& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, int& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, bool& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, std::vector<std::string>& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, std::vector<double>& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, std::vector<float>& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, std::vector<int>& output));
    MOCK_METHOD2(getParam, bool(const std::string& param_key, std::vector<bool>& output));
    ~MockParamServer() {};
};

ACTION_P(set_double, val)
{
  arg1 = val;
}

/**
 * Tests the constructor of the ConstraintChecker
 */ 
TEST(ConstraintChecker, constructor)
{

  // Test proper construction
  auto mock_param_server = std::make_shared<MockParamServer>();

  EXPECT_CALL(*mock_param_server, getParam("max_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  
  ASSERT_NO_THROW(ConstraintChecker cc(mock_param_server));


  // Test failing construction

  EXPECT_CALL(*mock_param_server, getParam("max_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(false)));

  ASSERT_THROW(ConstraintChecker cc(mock_param_server), std::invalid_argument);

  EXPECT_CALL(*mock_param_server, getParam("min_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(false)));

  ASSERT_THROW(ConstraintChecker cc(mock_param_server), std::invalid_argument);

  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(false)));

  ASSERT_THROW(ConstraintChecker cc(mock_param_server), std::invalid_argument);

  EXPECT_CALL(*mock_param_server, getParam("min_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(false)));

  ASSERT_THROW(ConstraintChecker cc(mock_param_server), std::invalid_argument);

  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(false)));

  ASSERT_THROW(ConstraintChecker cc(mock_param_server), std::invalid_argument);

  EXPECT_CALL(*mock_param_server, getParam("max_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(false)));

  ASSERT_THROW(ConstraintChecker cc(mock_param_server), std::invalid_argument);

  EXPECT_CALL(*mock_param_server, getParam("min_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(false)));

  ASSERT_THROW(ConstraintChecker cc(mock_param_server), std::invalid_argument);
}


/**
 * Tests the validateInitialState function of the ConstraintChecker
 */ 
TEST(ConstraintChecker, validateInitialState)
{

  // Build constraint checker
  auto mock_param_server = std::make_shared<MockParamServer>();

  EXPECT_CALL(*mock_param_server, getParam("max_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  std::unique_ptr<ConstraintChecker> cc;
  ASSERT_NO_THROW(cc = std::unique_ptr<ConstraintChecker>(new ConstraintChecker(mock_param_server)));

  // Test valid input state

  VehicleState vs; // All values default to 0

  ASSERT_NO_THROW(cc->validateInitialState(vs));

  // Test failing input states
  vs.steering_angle = 200.0;
  ASSERT_THROW(cc->validateInitialState(vs), std::invalid_argument);

  vs.steering_angle = -200.0;
  ASSERT_THROW(cc->validateInitialState(vs), std::invalid_argument);

  vs.steering_angle = 0.0; // Reset steer angle so it doesn't interfere with trailer angle check

  vs.trailer_angle = 200.0;
  ASSERT_THROW(cc->validateInitialState(vs), std::invalid_argument);

  vs.trailer_angle = -200.0;
  ASSERT_THROW(cc->validateInitialState(vs), std::invalid_argument);

  vs.trailer_angle = 0.0; // Reset trailer angle so it doesn't interfere with any future checks
}

/**
 * Tests the validateControlInputs function of the ConstraintChecker
 */ 
TEST(ConstraintChecker, validateControlInputs)
{

  // Build constraint checker
  auto mock_param_server = std::make_shared<MockParamServer>();

  EXPECT_CALL(*mock_param_server, getParam("max_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  std::unique_ptr<ConstraintChecker> cc;
  ASSERT_NO_THROW(cc = std::unique_ptr<ConstraintChecker>(new ConstraintChecker(mock_param_server)));

  VehicleState vs; // All values default to 0

  ASSERT_NO_THROW(cc->validateInitialState(vs)); // Ensure test is well formed and input state is valid

  // Test valid control input
  double timestep = 0.1;
  VehicleControlInput ci; // All values default to 0
  std::vector<VehicleControlInput> inputs;
  inputs.push_back(ci);
  inputs.push_back(ci);
  ASSERT_NO_THROW(cc->validateControlInputs(vs, inputs, timestep));

  // Test failing control inputs
  VehicleControlInput ci_bad;
  ci_bad.target_steering_angle = 200.0;
  inputs.push_back(ci_bad);
  ASSERT_THROW(cc->validateControlInputs(vs, inputs, timestep), std::invalid_argument);
  inputs.pop_back();

  ci_bad.target_steering_angle = -200.0;
  inputs.push_back(ci_bad);
  ASSERT_THROW(cc->validateControlInputs(vs, inputs, timestep), std::invalid_argument);
  inputs.pop_back();

  // Check steering angle rate 90 * .1 = 9
  ci_bad.target_steering_angle = 10.0;
  inputs.push_back(ci_bad);
  ASSERT_THROW(cc->validateControlInputs(vs, inputs, timestep), std::invalid_argument);
  inputs.pop_back();

  ci_bad.target_steering_angle = 0.0; // Reset steer angle so it doesn't interfere with other checks

  ci_bad.target_velocity = 20.0;
  inputs.push_back(ci_bad);
  ASSERT_THROW(cc->validateControlInputs(vs, inputs, timestep), std::invalid_argument);
  inputs.pop_back();

  ci_bad.target_velocity = -20.0;
  inputs.push_back(ci_bad);
  ASSERT_THROW(cc->validateControlInputs(vs, inputs, timestep), std::invalid_argument);
  inputs.pop_back();

  std::vector<VehicleControlInput> inputs_empty;
  ASSERT_THROW(cc->validateControlInputs(vs, inputs_empty, timestep), std::invalid_argument);
}
