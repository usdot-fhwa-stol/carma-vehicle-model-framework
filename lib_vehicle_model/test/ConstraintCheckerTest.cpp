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

  EXPECT_CALL(mock_param_server, getParam("max_forward_speed", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, 20.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("forward_acceleration_limit", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, 10.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("forward_deceleration_limit", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, -10.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("max_steering_angle", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, 180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("min_steering_angle", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, -180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("max_steering_angle_rate", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, 180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("max_trailer_angle", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, 180.0)), Return(true)));
  EXPECT_CALL(mock_param_server, getParam("min_trailer_angle", AnyOf(::testing::_))).WillOnce(DoAll(Invoke(set_double_ref(_, -180.0)), Return(true)));

  
  ConstraintChecker cc = new ConstraintChecker(parameter_server);
}

/**
 * Tests the validateInitialState function of the ConstraintChecker
 */ 
TEST(ConstraintChecker, validateInitialState)
{
  // Values 
  const double d = 43.6;
  const double a = 2.5;
  const double t = 2.4;
  const double v_i = 15.16666666;
  const double v_f = 21.16666666;
  const double error_bound = 0.0000001;


  // Solve for initial velocity
  // Find: v_i, Miss: v_f, Prop Order: a,d,t
  double v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::FINAL_VELOCITY, a,d,t);
  ASSERT_NEAR(v_i, v_i_result, error_bound);
  // Find: v_i, Miss: a,  Prop Order: v_f,d,t
  v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::ACCELERATION, v_f,d,t);
  ASSERT_NEAR(v_i, v_i_result, error_bound);
  // Find: v_i, Miss: d, Prop Order: v_f,a,t
  v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::DISTANCE, v_f,a,t);
  ASSERT_NEAR(v_i, v_i_result, error_bound);
  // Find: v_i, Miss: t, Prop Order: v_f,a,d
  v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::TIME, v_f,a,d);
  ASSERT_NEAR(v_i, v_i_result, error_bound);

  // Solve for final velocity
  // Find: v_f, Miss: v_i, Prop Order: a,d,t
  double v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::INITIAL_VELOCITY, a,d,t);
  ASSERT_NEAR(v_f, v_f_result, error_bound);
  // Find: v_f, Miss: a, Prop Order: v_i,d,t
  v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::ACCELERATION, v_i,d,t);
  ASSERT_NEAR(v_f, v_f_result, error_bound);
  // Find: v_f, Miss: d, Prop Order: v_i,a,t
  v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::DISTANCE, v_i,a,t);
  ASSERT_NEAR(v_f, v_f_result, error_bound);
  // Find: v_f, Miss: t, Prop Order: v_i,a,d
  v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::TIME, v_i,a,d);
  ASSERT_NEAR(v_f, v_f_result, error_bound);

  // Solve for acceleration
  // Find: a, Miss: v_i, Prop Order: v_f,d,t
  double a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::INITIAL_VELOCITY, v_f,d,t);
  ASSERT_NEAR(a, a_result, error_bound);
  // Find: a, Miss: v_f, Prop Order: v_i,d,t
  a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::FINAL_VELOCITY, v_i,d,t);
  ASSERT_NEAR(a, a_result, error_bound);
  // Find: a, Miss: d, Prop Order: v_i,v_f,t
  a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::DISTANCE, v_i,v_f,t);
  ASSERT_NEAR(a, a_result, error_bound);
  // Find: a, Miss: t, Prop Order: v_i,v_f,d
  a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::TIME, v_i,v_f,d);
  ASSERT_NEAR(a, a_result, error_bound);

  // Solve for distance
  // Find: d, Miss: v_i, Prop Order: v_f,a,t
  double d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::INITIAL_VELOCITY, v_f,a,t);
  ASSERT_NEAR(d, d_result, error_bound);
  // Find: d, Miss: v_f, Prop Order: v_i,a,t
  d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::FINAL_VELOCITY, v_i,a,t);
  ASSERT_NEAR(d, d_result, error_bound);
  // Find: d, Miss: a, Prop Order: v_i,v_f,t
  d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::ACCELERATION, v_i,v_f,t);
  ASSERT_NEAR(d, d_result, error_bound);
  // Find: d, Miss: t, Prop Order: v_i,v_f,a
  d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::TIME, v_i,v_f,a);
  ASSERT_NEAR(d, d_result, error_bound);

  // Solve for time
  // Find: t, Miss: v_i, Prop Order: v_f,a,d
  double t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::INITIAL_VELOCITY, v_f,a,d);
  ASSERT_NEAR(t, t_result, error_bound);
  // Find: t, Miss: v_f, Prop Order: v_i,a,d
  t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::FINAL_VELOCITY, v_i,a,d);
  ASSERT_NEAR(t, t_result, error_bound);
  // Find: t, Miss: a, Prop Order: v_i,v_f,d
  t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::ACCELERATION, v_i,v_f,d);
  ASSERT_NEAR(t, t_result, error_bound);
  // Find: t, Miss: d, Prop Order: v_i,v_f,a
  t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::DISTANCE, v_i,v_f,a);
  ASSERT_NEAR(t, t_result, error_bound);
}

// Run all the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
