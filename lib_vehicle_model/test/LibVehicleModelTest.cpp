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

#include <memory>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "lib_vehicle_model/LibVehicleModel.h"
#include "lib_vehicle_model/VehicleState.h"
#include "lib_vehicle_model/ParameterServer.h"
#include "lib_vehicle_model/ModelAccessException.h"

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

ACTION_P(set_string, val)
{
  arg1 = val;
}


/**
 * Tests the init function of the lib_vehicle_model namespace
 */ 
TEST(lib_vehicle_model, init)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  std::string path = std::string("test_libs/unittest_vehicle_model_shared_lib.so");

  EXPECT_CALL(*mock_param_server, getParam("vehicle_model_lib_path", A<std::string&>()))
    .WillRepeatedly(DoAll(set_string(path), Return(false))
  ); // Initially return false to check param access
  

  EXPECT_CALL(*mock_param_server, getParam("max_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(20.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("forward_acceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("forward_deceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  // Param for model to be loaded
  EXPECT_CALL(*mock_param_server, getParam("example_param", A<double&>())).WillOnce(DoAll(set_double(0.0), Return(true)));
  
  // Try loading without all params
  ASSERT_THROW(lib_vehicle_model::init(mock_param_server), std::invalid_argument);

  // Try loading a valid model
  EXPECT_CALL(*mock_param_server, getParam("vehicle_model_lib_path", A<std::string&>()))
    .WillRepeatedly(DoAll(set_string(path), Return(true))
  ); // Return true to check model loading

  ASSERT_NO_THROW(lib_vehicle_model::init(mock_param_server));

  // Try loading the model a second time
  ASSERT_THROW(lib_vehicle_model::init(mock_param_server), lib_vehicle_model::ModelAccessException);

  // Ensure the shared pointer for parameter server has been correctly set
  // Ref 1 - Test function scope
  // Ref 2 - lib_vehicle_model scope
  // Ref 3 - Loaded vehicle model scope
  ASSERT_EQ(3, mock_param_server.use_count());
  
  // Unload the vehicle model so we can run more tests
  lib_vehicle_model::unload();
  // Ensure the shared pointer for parameter server has been correctly set
  // Ref 1 - Test function scope
  ASSERT_EQ(1, mock_param_server.use_count());
}


/**
 * Tests the predict (no control input) function of the lib_vehicle_model namespace 
 */ 
TEST(lib_vehicle_model, predict_no_control)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  std::string path = std::string("test_libs/unittest_vehicle_model_shared_lib.so");

  EXPECT_CALL(*mock_param_server, getParam("vehicle_model_lib_path", A<std::string&>()))
    .WillRepeatedly(DoAll(set_string(path), Return(true))
  ); 

  EXPECT_CALL(*mock_param_server, getParam("max_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(20.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("forward_acceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("forward_deceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  // Param for model to be loaded
  EXPECT_CALL(*mock_param_server, getParam("example_param", A<double&>())).WillOnce(DoAll(set_double(0.0), Return(true)));
  
  // Test predict function exception before model load
  VehicleState vs; // All values default to 0

  ASSERT_THROW(lib_vehicle_model::predict(vs, 0.1, 1.0), lib_vehicle_model::ModelAccessException);

  // Try loading a valid model
  ASSERT_NO_THROW(lib_vehicle_model::init(mock_param_server));

  // Test predict function mis-matched timestep exception
  ASSERT_THROW(lib_vehicle_model::predict(vs, 1.0, 0.1), std::invalid_argument);

  // Test that constraint checker is called 
  vs.trailer_angle = -300.0;
  ASSERT_THROW(lib_vehicle_model::predict(vs, 0.1, 1.0), std::invalid_argument);
  vs.trailer_angle = 0.0;
  
  // Test valid prediction call
  ASSERT_NO_THROW(lib_vehicle_model::predict(vs, 0.1, 1.0));
  
  // Unload the vehicle model so we can run more tests
  unload();
  // Ensure the shared pointer for parameter server has been correctly set
  // Ref 1 - Test function scope
  ASSERT_EQ(1, mock_param_server.use_count());
}


/**
 * Tests the predict (with control input) function of the lib_vehicle_model namespace 
 */ 
TEST(lib_vehicle_model, predict_with_control)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  std::string path = std::string("test_libs/unittest_vehicle_model_shared_lib.so");

  EXPECT_CALL(*mock_param_server, getParam("vehicle_model_lib_path", A<std::string&>()))
    .WillRepeatedly(DoAll(set_string(path), Return(true))
  ); 

  EXPECT_CALL(*mock_param_server, getParam("max_forward_speed", A<double&>())).WillRepeatedly(DoAll(set_double(20.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("forward_acceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("forward_deceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_steering_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("max_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("min_trailer_angle", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  // Param for model to be loaded
  EXPECT_CALL(*mock_param_server, getParam("example_param", A<double&>())).WillOnce(DoAll(set_double(0.0), Return(true)));
  
  // Test predict function exception before model load
  VehicleState vs; // All values default to 0
  VehicleModelControlInput ci; // All values default to 0
  std::vector<VehicleModelControlInput> inputs;
  inputs.push_back(ci);
  inputs.push_back(ci);

  ASSERT_THROW(lib_vehicle_model::predict(vs, inputs, 0.1), lib_vehicle_model::ModelAccessException);

  // Try loading a valid model
  ASSERT_NO_THROW(lib_vehicle_model::init(mock_param_server));

  // Test that constraint checker is called for initial state
  vs.trailer_angle = -300.0;
  ASSERT_THROW(lib_vehicle_model::predict(vs, inputs, 0.1), std::invalid_argument);
  vs.trailer_angle = 0.0;

  // Test valid control input
  ASSERT_NO_THROW(lib_vehicle_model::predict(vs, inputs, 0.1));

  // Test that constraint checker is called for control inputs
  VehicleModelControlInput ci_bad;
  ci_bad.target_steering_angle = 200.0;
  inputs.push_back(ci_bad);
  ASSERT_THROW(lib_vehicle_model::predict(vs, inputs, 0.1), std::invalid_argument);
  inputs.pop_back();
  
  // Unload the vehicle model so we can run more tests
  unload();
  // Ensure the shared pointer for parameter server has been correctly set
  // Ref 1 - Test function scope
  ASSERT_EQ(1, mock_param_server.use_count());
}
