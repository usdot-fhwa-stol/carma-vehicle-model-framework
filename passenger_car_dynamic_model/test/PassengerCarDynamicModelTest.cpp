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
#include <iostream>
#include <sstream>
#include <cmath>
#include "lib_vehicle_model/VehicleControlInput.h"
#include "lib_vehicle_model/VehicleState.h"
#include "lib_vehicle_model/ParameterServer.h"
#include "lib_vehicle_model/LibVehicleModel.h"
#include "lib_vehicle_model/ODESolver.h"
#include "passenger_car_dynamic_model/PassengerCarDynamicModel.h"


/**
 * This file unit tests the setParameterServer checker class
 */ 

using ::testing::A;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::Unused;

class MockParamServer : public lib_vehicle_model::ParameterServer {
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
 * Tests the setParameterServer function of the PassengerCarDynamicModel class
 */ 
TEST(PassengerCarDynamicModel, setParameterServer)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model but with mass set to false
  EXPECT_CALL(*mock_param_server, getParam("length_to_f", A<double&>())).WillRepeatedly(DoAll(set_double(10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("length_to_r", A<double&>())).WillRepeatedly(DoAll(set_double(-10.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("effective_wheel_radius_f", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("effective_wheel_radius_r", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("tire_longitudinal_stiffness", A<double&>())).WillRepeatedly(DoAll(set_double(90.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("tire_cornering_stiffness", A<double&>())).WillRepeatedly(DoAll(set_double(180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("moment_of_inertia", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("vehicle_mass", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(false)));
  
  // Try building without all params
  PassengerCarDynamicModel pcm;

  ASSERT_THROW(pcm.setParameterServer(mock_param_server), std::invalid_argument);

  // Add mass to params
  EXPECT_CALL(*mock_param_server, getParam("vehicle_mass", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  // Try loading valid set of parameters
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));
}


/**
 * Tests the predict (no control input) function of the PassengerCarDynamicModel 
 */ 
TEST(lib_vehicle_model, predict_no_control)
{
    // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model but with mass set to false
  const double wheel_radius = 0.3048; //m
  const double length_to_tires = 2.4384;
  const double long_stiffness = 14166.0;
  const double lat_stiffness = 51560.0;
  const double moment_of_inertia = 2943.35411328;
  const double mass = 1302;

  EXPECT_CALL(*mock_param_server, getParam("length_to_f", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_tires), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("length_to_r", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_tires), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("effective_wheel_radius_f", A<double&>())).WillRepeatedly(DoAll(set_double(wheel_radius), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("effective_wheel_radius_r", A<double&>())).WillRepeatedly(DoAll(set_double(wheel_radius), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("tire_longitudinal_stiffness", A<double&>())).WillRepeatedly(DoAll(set_double(long_stiffness), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("tire_cornering_stiffness", A<double&>())).WillRepeatedly(DoAll(set_double(lat_stiffness), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("moment_of_inertia", A<double&>())).WillRepeatedly(DoAll(set_double(moment_of_inertia), Return(true)));
  EXPECT_CALL(*mock_param_server, getParam("vehicle_mass", A<double&>())).WillRepeatedly(DoAll(set_double(mass), Return(true)));
  
  // Try building without all params
  PassengerCarDynamicModel pcm;

  // Try loading valid set of parameters
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

  // Test no slip condition for 1 timestep
  lib_vehicle_model::VehicleState vs;
  vs.X_pos_global = 0;
  vs.Y_pos_global = 0;
  vs.orientation = 0;
  vs.longitudinal_vel = 5;
  vs.lateral_vel = 0;
  vs.yaw_rate = 0;
  vs.front_wheel_rotation_rate = vs.longitudinal_vel / wheel_radius;
  vs.rear_wheel_rotation_rate = vs.front_wheel_rotation_rate;
  vs.steering_angle = 0;
  vs.trailer_angle = 0;
  vs.prev_steering_cmd = 0;
  vs.prev_vel_cmd = vs.longitudinal_vel;

  // Build debug string for initial error
  std::vector<lib_vehicle_model::VehicleState> result = pcm.predict(vs,0.1,0.1);
  std::ostringstream msg;
  msg << "Size: " << result.size() << std::endl;;
  for (lib_vehicle_model::VehicleState v : result) {
    msg << v << std::endl;
  }

  // Check result size
  ASSERT_EQ(1, result.size());
  for (auto v: result) {
    ASSERT_NEAR(0.5, v.X_pos_global, 0.0000001) << msg.str();
    ASSERT_NEAR(vs.Y_pos_global, v.Y_pos_global, 0.0000001);
    ASSERT_NEAR(vs.orientation, v.orientation, 0.0000001);
    ASSERT_NEAR(vs.longitudinal_vel, v.longitudinal_vel, 0.0000001);
    ASSERT_NEAR(vs.lateral_vel, v.lateral_vel, 0.0000001);
    ASSERT_NEAR(vs.yaw_rate, v.yaw_rate, 0.0000001);
    ASSERT_NEAR(vs.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs.steering_angle, v.steering_angle, 0.0000001);
    ASSERT_NEAR(vs.trailer_angle, v.trailer_angle, 0.0000001);
    ASSERT_NEAR(vs.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
    ASSERT_NEAR(vs.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
  }


  // Test no slip condition for 5 timesteps
  result = pcm.predict(vs,0.1,0.5);
  // Check result size
  ASSERT_EQ(5, result.size());
  int i = 1;
  for (auto v: result) {
    ASSERT_NEAR(0.1 * i * vs.longitudinal_vel, v.X_pos_global, 0.0000001);
    ASSERT_NEAR(vs.Y_pos_global, v.Y_pos_global, 0.0000001);
    ASSERT_NEAR(vs.orientation, v.orientation, 0.0000001);
    ASSERT_NEAR(vs.longitudinal_vel, v.longitudinal_vel, 0.0000001);
    ASSERT_NEAR(vs.lateral_vel, v.lateral_vel, 0.0000001);
    ASSERT_NEAR(vs.yaw_rate, v.yaw_rate, 0.0000001);
    ASSERT_NEAR(vs.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs.steering_angle, v.steering_angle, 0.0000001);
    ASSERT_NEAR(vs.trailer_angle, v.trailer_angle, 0.0000001);
    ASSERT_NEAR(vs.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
    ASSERT_NEAR(vs.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
    i++;
  }


  // Test no slip condition for 5 timesteps with different orientation
  vs.orientation = M_PI_2; // Change orientation to be facing directly along y axis
  result = pcm.predict(vs,0.1,0.5);
  // Check result size
  ASSERT_EQ(5, result.size());
  i = 1;
  for (auto v: result) {
    ASSERT_NEAR(vs.X_pos_global, v.X_pos_global, 0.0000001);
    ASSERT_NEAR(0.1 * i * vs.longitudinal_vel, v.Y_pos_global, 0.0000001);
    ASSERT_NEAR(vs.orientation, v.orientation, 0.0000001);
    ASSERT_NEAR(vs.longitudinal_vel, v.longitudinal_vel, 0.0000001);
    ASSERT_NEAR(vs.lateral_vel, v.lateral_vel, 0.0000001);
    ASSERT_NEAR(vs.yaw_rate, v.yaw_rate, 0.0000001);
    ASSERT_NEAR(vs.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs.steering_angle, v.steering_angle, 0.0000001);
    ASSERT_NEAR(vs.trailer_angle, v.trailer_angle, 0.0000001);
    ASSERT_NEAR(vs.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
    ASSERT_NEAR(vs.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
    i++;
  }
  

  // Try stopped vehicle
  lib_vehicle_model::VehicleState vs_stop;
  result = pcm.predict(vs_stop,0.1,0.5);
  // Check result size
  ASSERT_EQ(5, result.size());
  i = 1;
  for (auto v: result) {
    ASSERT_NEAR(vs_stop.X_pos_global, v.X_pos_global, 0.0000001);
    ASSERT_NEAR(vs_stop.Y_pos_global, v.Y_pos_global, 0.0000001);
    ASSERT_NEAR(vs_stop.orientation, v.orientation, 0.0000001);
    ASSERT_NEAR(vs_stop.longitudinal_vel, v.longitudinal_vel, 0.0000001);
    ASSERT_NEAR(vs_stop.lateral_vel, v.lateral_vel, 0.0000001);
    ASSERT_NEAR(vs_stop.yaw_rate, v.yaw_rate, 0.0000001);
    ASSERT_NEAR(vs_stop.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs_stop.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
    ASSERT_NEAR(vs_stop.steering_angle, v.steering_angle, 0.0000001);
    ASSERT_NEAR(vs_stop.trailer_angle, v.trailer_angle, 0.0000001);
    ASSERT_NEAR(vs_stop.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
    ASSERT_NEAR(vs_stop.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
    i++;
  }
}


/**
 * Tests the predict (with control input) function of the PassengerCarDynamicModel 
 */ 
TEST(lib_vehicle_model, predict_with_control)
{

}
