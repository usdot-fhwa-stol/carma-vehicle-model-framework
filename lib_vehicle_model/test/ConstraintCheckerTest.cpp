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
#include "../src/lib_vehicle_model/ConstraintChecker.h"
#include "lib_vehicle_model/ParameterServer.h"


using namespace lib_vehicle_model;
using ::testing::A;
using ::testing::_;

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

void set_double_ref(double& var, double val)
{
  var = val;
}

/**
 * Tests the constructor of the ConstraintChecker
 */ 
TEST(ConstraintChecker, constructor)
{

  MockParamServer mock_param_server;

  EXPECT_CALL(mock_param_server, getParam("max_forward_speed", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), 20.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("forward_acceleration_limit", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), 10.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("forward_deceleration_limit", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), -10.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("max_steering_angle", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), 180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("min_steering_angle", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), -180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("max_steering_angle_rate", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), 180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("max_trailer_angle", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), 180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("min_trailer_angle", A<double&>())).WillOnce(DoAll(Invoke(set_double_ref(A<double&>(), -180.0)), Return(true)));

  
  ConstraintChecker cc = new ConstraintChecker(parameter_server);
}

// Run all the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
