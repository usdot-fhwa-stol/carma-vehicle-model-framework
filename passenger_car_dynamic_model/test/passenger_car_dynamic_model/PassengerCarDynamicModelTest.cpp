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
#include <iostream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <pacmod_msgs/WheelSpeedRpt.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <automotive_platform_msgs/SteeringFeedback.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "lib_vehicle_model/VehicleControlInput.h"
#include "lib_vehicle_model/VehicleState.h"
#include "lib_vehicle_model/ParameterServer.h"
#include "lib_vehicle_model/LibVehicleModel.h"
#include "lib_vehicle_model/ODESolver.h"
#include "passenger_car_dynamic_model/PassengerCarDynamicModel.h"
#include "model_test_tools/TestHelper.h"
#include "sigpack.h"


/**
 * This file unit tests the setParameterServer checker class
 * // TOP
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


void initializeParamServer (
  std::shared_ptr<MockParamServer> mock_param_server,
  double length_to_f,
  double length_to_r,
  double unloaded_wheel_radius_f,
  double unloaded_wheel_radius_r,
  double loaded_wheel_radius_f,
  double loaded_wheel_radius_r,
  double tire_longitudinal_stiffness_f,
  double tire_longitudinal_stiffness_r,
  double tire_cornering_stiffness_f,
  double tire_cornering_stiffness_r,
  double moment_of_inertia,
  double vehicle_mass,
  double steering_kP,
  double steering_kI,
  double steering_kD,
  double wheel_kP,
  double wheel_kI,
  double wheel_kD,
  double top_speed,
  double max_steering_rate
) {

    EXPECT_CALL(*mock_param_server, getParam("length_to_f", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_f), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("length_to_r", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_r), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("unloaded_wheel_radius_f", A<double&>())).WillRepeatedly(DoAll(set_double(unloaded_wheel_radius_f), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("unloaded_wheel_radius_r", A<double&>())).WillRepeatedly(DoAll(set_double(unloaded_wheel_radius_r), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("loaded_wheel_radius_f", A<double&>())).WillRepeatedly(DoAll(set_double(loaded_wheel_radius_f), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("loaded_wheel_radius_r", A<double&>())).WillRepeatedly(DoAll(set_double(loaded_wheel_radius_r), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("tire_longitudinal_stiffness_f", A<double&>())).WillRepeatedly(DoAll(set_double(tire_longitudinal_stiffness_f), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("tire_longitudinal_stiffness_r", A<double&>())).WillRepeatedly(DoAll(set_double(tire_longitudinal_stiffness_r), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("tire_cornering_stiffness_f", A<double&>())).WillRepeatedly(DoAll(set_double(tire_cornering_stiffness_f), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("tire_cornering_stiffness_r", A<double&>())).WillRepeatedly(DoAll(set_double(tire_cornering_stiffness_r), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("moment_of_inertia", A<double&>())).WillRepeatedly(DoAll(set_double(moment_of_inertia), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("vehicle_mass", A<double&>())).WillRepeatedly(DoAll(set_double(vehicle_mass), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("steering_kP", A<double&>())).WillRepeatedly(DoAll(set_double(steering_kP), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("steering_kI", A<double&>())).WillRepeatedly(DoAll(set_double(steering_kI), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("steering_kD", A<double&>())).WillRepeatedly(DoAll(set_double(steering_kD), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("wheel_kP", A<double&>())).WillRepeatedly(DoAll(set_double(wheel_kP), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("wheel_kI", A<double&>())).WillRepeatedly(DoAll(set_double(wheel_kI), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("wheel_kD", A<double&>())).WillRepeatedly(DoAll(set_double(wheel_kD), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("top_speed", A<double&>())).WillRepeatedly(DoAll(set_double(top_speed), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("max_steering_rate", A<double&>())).WillRepeatedly(DoAll(set_double(max_steering_rate), Return(true)));
}

/**
 * Tests the setParameterServer function of the PassengerCarDynamicModel class
 */ 
TEST(PassengerCarDynamicModel, DISABLED_setParameterServer)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  const double length_to_f = 1.24;
  const double length_to_r = 1.538;
  const double unloaded_wheel_radius_f = 0.355;
  const double unloaded_wheel_radius_r = 0.3625;
  const double loaded_wheel_radius_f = 0.355;
  const double loaded_wheel_radius_r = 0.3625;
  const double tire_longitudinal_stiffness_f = 90.0; // TODO
  const double tire_longitudinal_stiffness_r = 90.0; // TODO
  const double tire_cornering_stiffness_f = 116669.202;
  const double tire_cornering_stiffness_r = 93993.8288;
  const double moment_of_inertia = 4859.89;
  const double vehicle_mass = 2271.5;
  const double steering_kP = 1.0;
  const double steering_kI = 0.05;
  const double steering_kD = 0.0;
  const double wheel_kP = 0.1;
  const double wheel_kI = 0.0;
  const double wheel_kD = 0.0;
  double top_speed = 44.704;
  double max_steering_rate = 0.333333333;

  initializeParamServer(
    mock_param_server,
    length_to_f,
    length_to_r,
    unloaded_wheel_radius_f,
    unloaded_wheel_radius_r,
    loaded_wheel_radius_f,
    loaded_wheel_radius_r,
    tire_longitudinal_stiffness_f,
    tire_longitudinal_stiffness_r,
    tire_cornering_stiffness_f,
    tire_cornering_stiffness_r,
    moment_of_inertia,
    vehicle_mass,
    steering_kP,
    steering_kI,
    steering_kD,
    wheel_kP,
    wheel_kI,
    wheel_kD,
    top_speed,
    max_steering_rate
  );

  // Params for this vehicle model but with mass set to false
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
TEST(PassengerCarDynamicModel, DISABLED_predict_no_control)
{
    // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

// Params for this vehicle model
  const double length_to_f = 2.4384;
  const double length_to_r = 2.4384;
  const double unloaded_wheel_radius_f = 0.3048;
  const double unloaded_wheel_radius_r = 0.3048;
  const double loaded_wheel_radius_f = 0.3048;
  const double loaded_wheel_radius_r = 0.3048;
  const double tire_longitudinal_stiffness_f = 14166.0; // TODO
  const double tire_longitudinal_stiffness_r = 14166.0; // TODO
  const double tire_cornering_stiffness_f = 51560.0;
  const double tire_cornering_stiffness_r = 51560.0;
  const double moment_of_inertia = 2943.35411328;
  const double vehicle_mass = 1302;
  const double steering_kP = 0.0;
  const double steering_kI = 0.0;
  const double steering_kD = 0.0;
  const double wheel_kP = 0.0;
  const double wheel_kI = 0.0;
  const double wheel_kD = 0.0;
  double top_speed = 44.704;
  double max_steering_rate = 0.333333333;

  initializeParamServer(
    mock_param_server,
    length_to_f,
    length_to_r,
    unloaded_wheel_radius_f,
    unloaded_wheel_radius_r,
    loaded_wheel_radius_f,
    loaded_wheel_radius_r,
    tire_longitudinal_stiffness_f,
    tire_longitudinal_stiffness_r,
    tire_cornering_stiffness_f,
    tire_cornering_stiffness_r,
    moment_of_inertia,
    vehicle_mass,
    steering_kP,
    steering_kI,
    steering_kD,
    wheel_kP,
    wheel_kI,
    wheel_kD,
    top_speed,
    max_steering_rate
  );
  
  // Try building with all params
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
  vs.front_wheel_rotation_rate = vs.longitudinal_vel / loaded_wheel_radius_f; // In this unit test the effective radius = the loaded_wheel_radius
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
TEST(PassengerCarDynamicModel, DISABLED_predict_with_control)
{

}

// /**
//  * Tests the predict (with no control input using real data) function of the PassengerCarDynamicModel 
//  */ 
// TEST(PassengerCarDynamicModel, predict_no_control_real_data)
// {
//   // Setup param server
//   auto mock_param_server = std::make_shared<MockParamServer>();

//   // Params for this vehicle model
//   // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
//   const double length_to_f = 1.24;
//   const double length_to_r = 1.538;
//   const double effective_wheel_radius_f = 0.355;
//   const double effective_wheel_radius_r = 0.3625;
//   const double tire_longitudinal_stiffness_f = 90.0; // TODO
//   const double tire_longitudinal_stiffness_r = 90.0; // TODO
//   const double tire_cornering_stiffness_f = 116669.202;
//   const double tire_cornering_stiffness_r = 93993.8288;
//   const double moment_of_inertia = 4859.89;
//   const double vehicle_mass = 2271.5;

//   EXPECT_CALL(*mock_param_server, getParam("length_to_f", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_f), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("length_to_r", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_r), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("effective_wheel_radius_f", A<double&>())).WillRepeatedly(DoAll(set_double(effective_wheel_radius_f), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("effective_wheel_radius_r", A<double&>())).WillRepeatedly(DoAll(set_double(effective_wheel_radius_r), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("tire_longitudinal_stiffness_f", A<double&>())).WillRepeatedly(DoAll(set_double(tire_longitudinal_stiffness_f), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("tire_longitudinal_stiffness_r", A<double&>())).WillRepeatedly(DoAll(set_double(tire_longitudinal_stiffness_r), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("tire_cornering_stiffness_f", A<double&>())).WillRepeatedly(DoAll(set_double(tire_cornering_stiffness_f), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("tire_cornering_stiffness_r", A<double&>())).WillRepeatedly(DoAll(set_double(tire_cornering_stiffness_r), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("moment_of_inertia", A<double&>())).WillRepeatedly(DoAll(set_double(moment_of_inertia), Return(true)));
//   EXPECT_CALL(*mock_param_server, getParam("vehicle_mass", A<double&>())).WillRepeatedly(DoAll(set_double(vehicle_mass), Return(true)));


//   // Try building with all params
//   PassengerCarDynamicModel pcm;

//   // Try loading valid set of parameters
// //   try {
// // pcm.setParameterServer(mock_param_server);
// //   } catch (const std::exception& e) {
// //     std::cerr << e.what() << std::endl;
// //   }
//   ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

//   rosbag::Bag bag;
//   bag.open("test.bag", rosbag::bagmode::Read);

//   std::vector<std::string> topics;
//   topics.push_back(std::string("/localization/ndt_pose")); //geometry_msgs/PoseStamped
//   topics.push_back(std::string("pacmod/parsed_tx/wheel_speed_rpt")); //sensor_msgs/Imu
//   topics.push_back(std::string("gps/imu")); //pacmod_msgs/WheelSpeedRpt
//   topics.push_back(std::string("vehicle/twist")); // geometry_msgs/TwistStamped

//   rosbag::View view(bag, rosbag::TopicQuery(topics));

//   BagSubscriber<geometry_msgs::PoseStamped> pose_sub;
//   BagSubscriber<sensor_msgs::Imu> imu_sub;
//   BagSubscriber<pacmod_msgs::WheelSpeedRpt> wheel_sub;
//   BagSubscriber<geometry_msgs::TwistStamped> twist_sub;


//   message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::Imu, pacmod_msgs::WheelSpeedRpt, geometry_msgs::TwistStamped>
//     sync(pose_sub, imu_sub, wheel_sub, twist_sub, 25);

//   sync.registerCallback(boost::bind(&vehicleStateSyncCallback, _1, _2, _3, _4));

//   std::vector<geometry_msgs::PoseStampedConstPtr> veh_poses;
//   veh_poses.reserve(2400);

//   for(const rosbag::MessageInstance m : view)
//   {
//     geometry_msgs::PoseStampedConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
//     if (pose != NULL)
//         veh_poses.push_back(pose);

//     // std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//     // if (i != NULL)
//     //     std::cout << i->data << std::endl;
//   }

//   bag.close();

//   // TODO we need some logic to sync the pose with current wheel reports

//   for (geometry_msgs::PoseStampedConstPtr msg : veh_poses) {
//     lib_vehicle_model::VehicleState vs;
//     vs.X_pos_global = msg->pose.position.x;
//     vs.Y_pos_global = msg->pose.position.y;
//     vs.orientation = getYawFromPose(msg);
//     vs.longitudinal_vel = 5;
//     vs.lateral_vel = 0; // TODO fill out
//     vs.yaw_rate = 0;
//    // vs.front_wheel_rotation_rate = vs.longitudinal_vel / wheel_radius;
//     vs.rear_wheel_rotation_rate = vs.front_wheel_rotation_rate;
//     vs.steering_angle = 0;
//     vs.trailer_angle = 0;
//     vs.prev_steering_cmd = 0; // TODO Decide how best to populate the prev cmds It might not be needed at this point 
//     vs.prev_vel_cmd = vs.longitudinal_vel;
//   }

// }
// STEER
class SteeringStateFunctor {
  model_test_tools::SteeringTestHelper& helper_;
  public:
    SteeringStateFunctor(model_test_tools::SteeringTestHelper& helper) : helper_(helper)
    {}

    // Callback for ode observer during integration
    void operator()(
      const autoware_msgs::VehicleCmdConstPtr cmd_msg,
      const automotive_platform_msgs::SteeringFeedbackConstPtr feedback_msg
    ) {
      const double radPerSteeringRad = 0.05922;
      double stamp = cmd_msg->header.stamp.toSec();
      double setpoint = cmd_msg->ctrl_cmd.steering_angle;
      double current = feedback_msg->steering_wheel_angle * radPerSteeringRad;

      helper_.steeringCallback(stamp, setpoint, current);
    }
};

/**
 * Tests the prediction of steering angle
 * 
 * NOTE: To view the data from this unit test set the steer_test_helper(true) for one rosbag at a time and look at the steering_sync.csv file
 */ 
TEST(PassengerCarDynamicModel, DISABLED_evaluate_steering_pred_accuracy)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
  const double length_to_f = 1.24;
  const double length_to_r = 1.538;
  const double unloaded_wheel_radius_f = 0.38354;
  const double unloaded_wheel_radius_r = 0.38354;
  const double loaded_wheel_radius_f = 0.355;
  const double loaded_wheel_radius_r = 0.3625;
  const double tire_longitudinal_stiffness_f = 90.0; // TODO
  const double tire_longitudinal_stiffness_r = 90.0; // TODO
  const double tire_cornering_stiffness_f = 116669.202;
  const double tire_cornering_stiffness_r = 93993.8288;
  const double moment_of_inertia = 4859.89;
  const double vehicle_mass = 2271.5;
  // NOTE: If steering model is changed then the steering PID must change as well
  const double steering_kP = 1.0;
  const double steering_kI = 0.05;
  const double steering_kD = 0.0;

  const double wheel_kP = 0.1;
  const double wheel_kI = 0.0;
  const double wheel_kD = 0.0;
  double top_speed = 44.704;
  double max_steering_rate = 0.333333333;

  initializeParamServer(
    mock_param_server,
    length_to_f,
    length_to_r,
    unloaded_wheel_radius_f,
    unloaded_wheel_radius_r,
    loaded_wheel_radius_f,
    loaded_wheel_radius_r,
    tire_longitudinal_stiffness_f,
    tire_longitudinal_stiffness_r,
    tire_cornering_stiffness_f,
    tire_cornering_stiffness_r,
    moment_of_inertia,
    vehicle_mass,
    steering_kP,
    steering_kI,
    steering_kD,
    wheel_kP,
    wheel_kI,
    wheel_kD,
    top_speed,
    max_steering_rate
  );

  // Try building with all params
  PassengerCarDynamicModel pcm;
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));


  //// 
  // Open Data File 1
  ////
  model_test_tools::SteeringTestHelper steer_test_helper(false, 0.05);

  rosbag::Bag bag;
  bag.open("data/_2019-09-30-12-19-17_steer_new_test1.bag", rosbag::bagmode::Read);
  
  std::string vehicle_cmd_tpc = "/vehicle_cmd";
  std::string steering_feedback_tpc = "/steering_feedback";
  std::vector<std::string> topics;
  topics.push_back(vehicle_cmd_tpc); 
  topics.push_back(steering_feedback_tpc);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  model_test_tools::BagSubscriber<autoware_msgs::VehicleCmd> cmd_sub(vehicle_cmd_tpc);
  model_test_tools::BagSubscriber<automotive_platform_msgs::SteeringFeedback> steering_sub(steering_feedback_tpc);

  typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::VehicleCmd, automotive_platform_msgs::SteeringFeedback> ApproxTimePolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(10), cmd_sub, steering_sub);
  
  SteeringStateFunctor steer_functor(steer_test_helper);
  sync.registerCallback(boost::bind<void>(steer_functor, _1, _2));

  // Playback bag file to collect data
  for(const rosbag::MessageInstance m : view)
  {
    cmd_sub.onMessageInstance(m);
    steering_sub.onMessageInstance(m);
  }

  bag.close();

  std::tuple<double,double> max_error_and_rmse = steer_test_helper.evaluateSteeringData(pcm);
  double maxError = std::get<0>(max_error_and_rmse);
  double rmse = std::get<1>(max_error_and_rmse);

  ASSERT_LE(rmse, 0.03) << " RMSE larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  ASSERT_LE(maxError, 0.12015) << " MaxError larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  
  steer_test_helper.logData(steer_test_helper.lastForcast);
  steer_test_helper.sync_csv_file.close();



  //// 
  // Open Data File 2
  ////
  model_test_tools::SteeringTestHelper steer_test_helper2(false, 0.05);

  rosbag::Bag bag2;
  bag2.open("data/_2019-09-30-12-22-31_steer_new_test3.bag", rosbag::bagmode::Read);
  
  rosbag::View view2(bag2, rosbag::TopicQuery(topics));

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<ApproxTimePolicy> sync2(ApproxTimePolicy(10), cmd_sub, steering_sub);
  SteeringStateFunctor steer_functor2(steer_test_helper2);
  sync2.registerCallback(boost::bind<void>(steer_functor2, _1, _2));

  // Playback bag file to collect data
  for(const rosbag::MessageInstance m : view2)
  {
    cmd_sub.onMessageInstance(m);
    steering_sub.onMessageInstance(m);
  }

  bag2.close();

  max_error_and_rmse = steer_test_helper2.evaluateSteeringData(pcm);
  maxError = std::get<0>(max_error_and_rmse);
  rmse = std::get<1>(max_error_and_rmse);

  ASSERT_LE(rmse, 0.021) << " RMSE larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  ASSERT_LE(maxError, 0.076) << " MaxError larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  
  steer_test_helper2.logData(steer_test_helper2.lastForcast);
  steer_test_helper2.sync_csv_file.close();
}


class VehicleStateFunctor {
  model_test_tools::ModelTestHelper& helper_;
  public:
    VehicleStateFunctor(model_test_tools::ModelTestHelper& helper) : helper_(helper)
    {}

    // Callback for ode observer during integration
    void operator()(const geometry_msgs::PoseStampedConstPtr pose_msg,
    const sensor_msgs::ImuConstPtr imu_msg,
    const pacmod_msgs::WheelSpeedRptConstPtr wheel_msg,
    const geometry_msgs::TwistStampedConstPtr twist_msg,
    const autoware_msgs::VehicleCmdConstPtr cmd_msg,
    const automotive_platform_msgs::SteeringFeedbackConstPtr steer_msg)
    {
      const double radPerSteeringRad = 0.05922; // This was computed experimentally however it matches the the AStuff json file's 0.51/8.6 = 0.0593
      double stamp = pose_msg->header.stamp.toSec();
      lib_vehicle_model::VehicleState vs;
      vs.X_pos_global = pose_msg->pose.position.x;
      vs.Y_pos_global = pose_msg->pose.position.y;
      double roll, pitch, yaw;
      model_test_tools::getRPYFromPose(pose_msg->pose, roll, pitch, yaw);

      vs.orientation = yaw;
      vs.longitudinal_vel = twist_msg->twist.linear.x;
      vs.lateral_vel = 0;

      vs.yaw_rate = imu_msg->angular_velocity.z;
      vs.front_wheel_rotation_rate = (wheel_msg->front_left_wheel_speed + wheel_msg->front_right_wheel_speed) / 2.0; // Front wheel speed average
      vs.rear_wheel_rotation_rate = (wheel_msg->rear_left_wheel_speed + wheel_msg->rear_right_wheel_speed) / 2.0; // Front wheel speed average
      vs.steering_angle = steer_msg->steering_wheel_angle * radPerSteeringRad; // Convert steering wheel angle to wheel angle

      lib_vehicle_model::VehicleControlInput ctrl_input;
      ctrl_input.target_steering_angle = cmd_msg->ctrl_cmd.steering_angle;
      ctrl_input.target_velocity = cmd_msg->ctrl_cmd.linear_velocity;

      helper_.vehicleStateCallback(stamp, vs, ctrl_input); 
    }
};

/**
 * This Unit test is for finding initial PID values based on bag files.
 * The determined values should not be taken as guaranteed optimal
 * Instead they provide a reasonable starting point for tunning. 
 * 
 * NOTE: This test is currently disabled. To enable it remove the DISABLED_ prefix from the test name
 */ 
TEST(PassengerCarDynamicModel, DISABLED_find_steering_pid)
{

  // Create test helper. 
  // NOTE: Set to true to record data to csv file
  model_test_tools::SteeringTestHelper steer_test_helper(false, 0.05); 

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
  const double length_to_f = 1.24;
  const double length_to_r = 1.538;
  const double unloaded_wheel_radius_f = 0.38354;
  const double unloaded_wheel_radius_r = 0.38354;
  const double loaded_wheel_radius_f = 0.355;
  const double loaded_wheel_radius_r = 0.3625;
  const double tire_longitudinal_stiffness_f = 90.0; // TODO
  const double tire_longitudinal_stiffness_r = 90.0; // TODO
  const double tire_cornering_stiffness_f = 116669.202;
  const double tire_cornering_stiffness_r = 93993.8288;
  const double moment_of_inertia = 4859.89;
  const double vehicle_mass = 2271.5;
  // NOTE: If steering model is changed then the steering PID must change as well
  const double steering_kP = 1.0;
  const double steering_kI = 0.05;
  const double steering_kD = 0.0;

  const double wheel_kP = 0.1;
  const double wheel_kI = 0.0;
  const double wheel_kD = 0.0;
  double top_speed = 44.704;
  double max_steering_rate = 0.333333333;

  initializeParamServer(
    mock_param_server,
    length_to_f,
    length_to_r,
    unloaded_wheel_radius_f,
    unloaded_wheel_radius_r,
    loaded_wheel_radius_f,
    loaded_wheel_radius_r,
    tire_longitudinal_stiffness_f,
    tire_longitudinal_stiffness_r,
    tire_cornering_stiffness_f,
    tire_cornering_stiffness_r,
    moment_of_inertia,
    vehicle_mass,
    steering_kP,
    steering_kI,
    steering_kD,
    wheel_kP,
    wheel_kI,
    wheel_kD,
    top_speed,
    max_steering_rate
  );

  // Try building with all params
  PassengerCarDynamicModel pcm;
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

  rosbag::Bag bag;
  bag.open("data/_2019-09-30-12-19-17_steer_new_test1.bag", rosbag::bagmode::Read);
  
  std::string vehicle_cmd_tpc = "/vehicle_cmd";
  std::string steering_feedback_tpc = "/steering_feedback";
  std::vector<std::string> topics;
  topics.push_back(vehicle_cmd_tpc); 
  topics.push_back(steering_feedback_tpc);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  model_test_tools::BagSubscriber<autoware_msgs::VehicleCmd> cmd_sub(vehicle_cmd_tpc);
  model_test_tools::BagSubscriber<automotive_platform_msgs::SteeringFeedback> steering_sub(steering_feedback_tpc);

  typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::VehicleCmd, automotive_platform_msgs::SteeringFeedback> ApproxTimePolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(10), cmd_sub, steering_sub);
  SteeringStateFunctor steer_functor(steer_test_helper);
  sync.registerCallback(boost::bind<void>(steer_functor, _1, _2));
  // Playback bag file to collect data
  for(const rosbag::MessageInstance m : view)
  {
    cmd_sub.onMessageInstance(m);
    steering_sub.onMessageInstance(m);
  }

  bag.close();

  std::vector<double> bestForcast;
  double minRMSE = 0;
  double bestP = 0;
  double bestI = 0;
  double bestD = 0;
  double minMaxError = 0;
  double minAllowableMaxError = 0.175;
  bool first = true;
  for (double p = 0.0; p < 3.0; p += 0.1) { // P
    if (-0.0000001 < p && p <0.0000001) { // Skip 0
       continue;
    } 
    EXPECT_CALL(*mock_param_server, getParam("steering_kP", A<double&>())).WillRepeatedly(DoAll(set_double(p), Return(true)));

    for (double i = -0.2; i < 0.2; i += 0.05) { // I
      EXPECT_CALL(*mock_param_server, getParam("steering_kI", A<double&>())).WillRepeatedly(DoAll(set_double(i), Return(true)));

      for (double d = -0.5; d < 0.5; d += 0.05) { // D
        EXPECT_CALL(*mock_param_server, getParam("steering_kD", A<double&>())).WillRepeatedly(DoAll(set_double(d), Return(true)));

        pcm.setParameterServer(mock_param_server);
        
        std::tuple<double,double> max_error_and_rmse = steer_test_helper.evaluateSteeringData(pcm);
        double maxError = std::get<0>(max_error_and_rmse);
        double rmse = std::get<1>(max_error_and_rmse);

        if (first) {
          first = false;
          minRMSE = rmse;
          bestP = p;
          bestI = i;
          bestD = d;
          bestForcast = steer_test_helper.lastForcast;
          minMaxError = maxError;
        } else if (rmse < minRMSE && (minMaxError > maxError || maxError < minAllowableMaxError)) {
          minRMSE = rmse;
          bestP = p;
          bestI = i;
          bestD = d;
          bestForcast = steer_test_helper.lastForcast;
          minMaxError = maxError;
        }
      }
    }
    std::cerr << "Current P " << p << std::endl;
    std::cerr << "CurBest P: " <<  bestP << " I: " << bestI << " D: " << bestD << std::endl;
    std::cerr << "CurBest RMSE: " << minRMSE << std::endl;
    std::cerr << "CurBest MaxError: " << minMaxError << std::endl;
  }
  steer_test_helper.logData(bestForcast);
  steer_test_helper.sync_csv_file.close();
  std::cerr << std::endl << "Best P: " <<  bestP << " I: " << bestI << " D: " << bestD << std::endl;
  std::cerr << "Best RMSE: " << minRMSE << std::endl;
  std::cerr << "Best MaxError: " << minMaxError << std::endl;
}

/**
 * Tests the overall prediction performance of the model
 * 
 * NOTE: To view the data from this unit test set the steer_test_helper(true) for one rosbag at a time and look at the steering_sync.csv file
 */ 
TEST(PassengerCarDynamicModel, evaluate_overall_pred)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
  const double length_to_f = 1.24;
  const double length_to_r = 1.538;
  const double unloaded_wheel_radius_f = 0.38354;
  const double unloaded_wheel_radius_r = 0.38354;
  const double loaded_wheel_radius_f = 0.355;
  const double loaded_wheel_radius_r = 0.3625;
  const double tire_longitudinal_stiffness_f = 90.0; // TODO
  const double tire_longitudinal_stiffness_r = 90.0; // TODO
  const double tire_cornering_stiffness_f = 116669.202;
  const double tire_cornering_stiffness_r = 93993.8288;
  const double moment_of_inertia = 4859.89;
  const double vehicle_mass = 2271.5;
  // NOTE: If steering model is changed then the steering PID must change as well
  const double steering_kP = 1.0;
  const double steering_kI = 0.05;
  const double steering_kD = 0.0;

  const double wheel_kP = 0.7;
  const double wheel_kI = 0.0;
  const double wheel_kD = 0.0;
  double top_speed = 44.704;
  double max_steering_rate = 0.333333333;

  initializeParamServer(
    mock_param_server,
    length_to_f,
    length_to_r,
    unloaded_wheel_radius_f,
    unloaded_wheel_radius_r,
    loaded_wheel_radius_f,
    loaded_wheel_radius_r,
    tire_longitudinal_stiffness_f,
    tire_longitudinal_stiffness_r,
    tire_cornering_stiffness_f,
    tire_cornering_stiffness_r,
    moment_of_inertia,
    vehicle_mass,
    steering_kP,
    steering_kI,
    steering_kD,
    wheel_kP,
    wheel_kI,
    wheel_kD,
    top_speed,
    max_steering_rate
  );

  // Try building with all params
  PassengerCarDynamicModel pcm;
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));


  //// 
  // Open Data File
  ////
  model_test_tools::ModelTestHelper model_test_helper(true, 0.01, 6.0);

  rosbag::Bag bag;
  bag.open("data/_2019-10-03-17-11-49_full_loop_validation_filtered.bag", rosbag::bagmode::Read);
  
  std::string imu_tpc               = "/hardware_interface/imu_raw";
  std::string wheel_rpt_tpc         = "/hardware_interface/pacmod/parsed_tx/wheel_speed_rpt";
  std::string steering_feedback_tpc = "/hardware_interface/steering_feedback";
  std::string twist_tpc             = "/hardware_interface/vehicle/twist";
  std::string vehicle_cmd_tpc       = "/hardware_interface/vehicle_cmd";
  std::string pose_tpc              = "/localization/ndt_pose";

  std::vector<std::string> topics;
  topics.push_back(imu_tpc); 
  topics.push_back(wheel_rpt_tpc);
  topics.push_back(steering_feedback_tpc);
  topics.push_back(twist_tpc);
  topics.push_back(vehicle_cmd_tpc);
  topics.push_back(pose_tpc);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  model_test_tools::BagSubscriber<sensor_msgs::Imu> imu_sub(imu_tpc);
  model_test_tools::BagSubscriber<pacmod_msgs::WheelSpeedRpt> wheel_sub(wheel_rpt_tpc);
  model_test_tools::BagSubscriber<automotive_platform_msgs::SteeringFeedback> steer_sub(steering_feedback_tpc);
  model_test_tools::BagSubscriber<geometry_msgs::TwistStamped> twist_sub(twist_tpc);
  model_test_tools::BagSubscriber<autoware_msgs::VehicleCmd> cmd_sub(vehicle_cmd_tpc);
  model_test_tools::BagSubscriber<geometry_msgs::PoseStamped> pose_sub(pose_tpc);

  typedef message_filters::sync_policies::ApproximateTime<
    geometry_msgs::PoseStamped, sensor_msgs::Imu, pacmod_msgs::WheelSpeedRpt,
    geometry_msgs::TwistStamped, autoware_msgs::VehicleCmd, automotive_platform_msgs::SteeringFeedback
    > ApproxTimePolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), pose_sub, imu_sub, wheel_sub, twist_sub, cmd_sub, steer_sub);
  VehicleStateFunctor veh_functor(model_test_helper);
  sync.registerCallback(boost::bind<void>(veh_functor, _1, _2, _3, _4, _5, _6));

  // Playback bag file to collect data
  for(const rosbag::MessageInstance m : view)
  {

    imu_sub.onMessageInstance(m);
    pose_sub.onMessageInstance(m);
    wheel_sub.onMessageInstance(m);
    twist_sub.onMessageInstance(m);
    cmd_sub.onMessageInstance(m);
    steer_sub.onMessageInstance(m);
  }

  bag.close();

  //std::tuple<double,double> max_error_and_rmse = steer_test_helper.evaluateSteeringData(pcm);
  model_test_helper.evaluateData(pcm);
  // double maxError = std::get<0>(max_error_and_rmse);
  // double rmse = std::get<1>(max_error_and_rmse);

  // ASSERT_LE(rmse, 0.03) << " RMSE larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  // ASSERT_LE(maxError, 0.12015) << " MaxError larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  
  model_test_helper.logData(model_test_helper.last_forcast_);
  model_test_helper.sync_csv_file.close();
}


class SpeedStateFunctor {
  model_test_tools::SpeedTestHelper& helper_;
  public:
    SpeedStateFunctor(model_test_tools::SpeedTestHelper& helper) : helper_(helper)
    {}

    // Callback for ode observer during integration
    void operator()(
    const sensor_msgs::ImuConstPtr imu_msg, // Used for acceleration
    const pacmod_msgs::WheelSpeedRptConstPtr wheel_msg, // Used for wheel speed
    const autoware_msgs::VehicleCmdConstPtr cmd_msg, // Used for command speed
    const gps_common::GPSFixConstPtr fix_msg // Used for ground speed
  ) {

    double stamp = fix_msg->header.stamp.toSec(); // Get Stamp

    double cmd_vel = cmd_msg->ctrl_cmd.linear_velocity; // Get cmd_vel

    model_test_tools::SpeedState ss; // Get speed state
    ss.longitudinal_accel = imu_msg->linear_acceleration.y; // TODO this changes to x or y depending on car
    ss.front_wheel_angular_vel = (wheel_msg->front_left_wheel_speed + wheel_msg->front_right_wheel_speed) / 2.0;
    ss.rear_wheel_angular_vel = (wheel_msg->rear_left_wheel_speed + wheel_msg->rear_right_wheel_speed) / 2.0;
    ss.longitudinal_vel = fix_msg->speed; // NOTE: This assumes forward driving

    // Store data
    helper_.syncCallback(stamp, ss, cmd_vel);
  }
};

/**
 * Tests the overall prediction performance of the model
 * 
 * NOTE: To view the data from this unit test set the steer_test_helper(true) for one rosbag at a time and look at the steering_sync.csv file
 */ 
TEST(PassengerCarDynamicModel, DISABLED_evaluate_overall_pred_todo)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
  const double length_to_f = 1.24;
  const double length_to_r = 1.538;
  const double unloaded_wheel_radius_f = 0.38354;
  const double unloaded_wheel_radius_r = 0.38354;
  const double loaded_wheel_radius_f = 0.355;
  const double loaded_wheel_radius_r = 0.3625;
  const double tire_longitudinal_stiffness_f = 90.0; // TODO
  const double tire_longitudinal_stiffness_r = 90.0; // TODO
  const double tire_cornering_stiffness_f = 116669.202;
  const double tire_cornering_stiffness_r = 93993.8288;
  const double moment_of_inertia = 4859.89;
  const double vehicle_mass = 2271.5;
  // NOTE: If steering model is changed then the steering PID must change as well
  const double steering_kP = 1.0;
  const double steering_kI = 0.05;
  const double steering_kD = 0.0;

  const double wheel_kP = 0.7;
  const double wheel_kI = 0.0;
  const double wheel_kD = 0.0;
  double top_speed = 44.704;
  double max_steering_rate = 0.333333333;

  initializeParamServer(
    mock_param_server,
    length_to_f,
    length_to_r,
    unloaded_wheel_radius_f,
    unloaded_wheel_radius_r,
    loaded_wheel_radius_f,
    loaded_wheel_radius_r,
    tire_longitudinal_stiffness_f,
    tire_longitudinal_stiffness_r,
    tire_cornering_stiffness_f,
    tire_cornering_stiffness_r,
    moment_of_inertia,
    vehicle_mass,
    steering_kP,
    steering_kI,
    steering_kD,
    wheel_kP,
    wheel_kI,
    wheel_kD,
    top_speed,
    max_steering_rate
  );

  // Try building with all params
  PassengerCarDynamicModel pcm;
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

  //// 
  // Open Data File
  ////
  model_test_tools::SpeedTestHelper speed_test_helper(true);

  rosbag::Bag bag;
  bag.open("data/_2019-10-30-12-06-04_profile.bag", rosbag::bagmode::Read);
  
  std::string imu_tpc         = "/imu_raw";
  std::string wheel_rpt_tpc   = "/pacmod/parsed_tx/wheel_speed_rpt";
  std::string vehicle_cmd_tpc = "/vehicle_cmd";
  std::string gps_tpc         = "/gnss_fix_fused";

  std::vector<std::string> topics;
  topics.push_back(imu_tpc); 
  topics.push_back(wheel_rpt_tpc);
  topics.push_back(vehicle_cmd_tpc);
  topics.push_back(gps_tpc);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  model_test_tools::BagSubscriber<sensor_msgs::Imu> imu_sub(imu_tpc);
  model_test_tools::BagSubscriber<pacmod_msgs::WheelSpeedRpt> wheel_sub(wheel_rpt_tpc);
  model_test_tools::BagSubscriber<autoware_msgs::VehicleCmd> cmd_sub(vehicle_cmd_tpc);
  model_test_tools::BagSubscriber<gps_common::GPSFix> gps_sub(gps_tpc);

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Imu, pacmod_msgs::WheelSpeedRpt,
    autoware_msgs::VehicleCmd, gps_common::GPSFix
    > ApproxTimePolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), imu_sub, wheel_sub, cmd_sub, gps_sub);
  SpeedStateFunctor functor(speed_test_helper);
  sync.registerCallback(boost::bind<void>(functor, _1, _2, _3, _4));


  //
  // Create Low Pass IIR filter for IMU acceleration data. Based on butterworth filter in matlab
  // Order: 2nd
  // Passband Freq: 21 Hz
  sp::IIR_filt<double,double,double> imu_filter;
  arma::vec3 b;
  b[0] = 0.222955;
  b[1] = 0.445910;
  b[2] = 0.222955;

  arma::vec3 a;
  a[0] = 1.000000;
  a[1] = -0.295200;
  a[2] = 0.187020;

  imu_filter.set_coeffs(b, a);

  // Playback bag file to collect data
  for(const rosbag::MessageInstance m : view)
  {
    // Filter IMU data. This should be done before going into model_test_tools so that the time_synchronizer does not impact the filter
    sensor_msgs::ImuConstPtr s = m.instantiate<sensor_msgs::Imu>();
    sensor_msgs::Imu newMsg;
    if (s != NULL) { //(TODO this requires filtering )
        newMsg.header = s->header;
        newMsg.angular_velocity = s->angular_velocity;
        newMsg.angular_velocity_covariance = s->angular_velocity_covariance;
        newMsg.orientation = s->orientation;
        newMsg.orientation_covariance = s->orientation_covariance;
        newMsg.linear_acceleration = s->linear_acceleration;
        newMsg.linear_acceleration_covariance = s->linear_acceleration_covariance;
        newMsg.linear_acceleration.x = imu_filter(s->linear_acceleration.x);
    }

    sensor_msgs::ImuConstPtr imu_ptr(new sensor_msgs::Imu(newMsg));

    imu_sub.onMessage(imu_ptr);
    wheel_sub.onMessageInstance(m);
    cmd_sub.onMessageInstance(m);
    gps_sub.onMessageInstance(m);
  }

  bag.close();

  //std::tuple<double,double> max_error_and_rmse = steer_test_helper.evaluateSteeringData(pcm);
  speed_test_helper.evaluateData(pcm);
  // double maxError = std::get<0>(max_error_and_rmse);
  // double rmse = std::get<1>(max_error_and_rmse);

  // ASSERT_LE(rmse, 0.03) << " RMSE larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  // ASSERT_LE(maxError, 0.12015) << " MaxError larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  
  speed_test_helper.logData(speed_test_helper.last_forcast_);
  speed_test_helper.sync_csv_file.close();
}


/**
 * This Unit test is for finding initial PID values based on bag files.
 * The determined values should not be taken as guaranteed optimal
 * Instead they provide a reasonable starting point for tunning. 
 * 
 * NOTE: This test is currently disabled. To enable it remove the DISABLED_ prefix from the test name
 */ 
TEST(PassengerCarDynamicModel, DISABLED_find_speed_pid)
{

  // Create test helper. 
  // NOTE: Set to true to record data to csv file
  model_test_tools::SpeedTestHelper speed_test_helper(true); 

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
  const double length_to_f = 1.24;
  const double length_to_r = 1.538;
  const double unloaded_wheel_radius_f = 0.38354;
  const double unloaded_wheel_radius_r = 0.38354;
  const double loaded_wheel_radius_f = 0.355;
  const double loaded_wheel_radius_r = 0.3625;
  const double tire_longitudinal_stiffness_f = 90.0; // TODO
  const double tire_longitudinal_stiffness_r = 90.0; // TODO
  const double tire_cornering_stiffness_f = 116669.202;
  const double tire_cornering_stiffness_r = 93993.8288;
  const double moment_of_inertia = 4859.89;
  const double vehicle_mass = 2271.5;
  // NOTE: If steering model is changed then the steering PID must change as well
  const double steering_kP = 1.0;
  const double steering_kI = 0.05;
  const double steering_kD = 0.0;

  const double wheel_kP = 0.7;
  const double wheel_kI = 0.0;
  const double wheel_kD = 0.0;
  double top_speed = 44.704;
  double max_steering_rate = 0.333333333;

  initializeParamServer(
    mock_param_server,
    length_to_f,
    length_to_r,
    unloaded_wheel_radius_f,
    unloaded_wheel_radius_r,
    loaded_wheel_radius_f,
    loaded_wheel_radius_r,
    tire_longitudinal_stiffness_f,
    tire_longitudinal_stiffness_r,
    tire_cornering_stiffness_f,
    tire_cornering_stiffness_r,
    moment_of_inertia,
    vehicle_mass,
    steering_kP,
    steering_kI,
    steering_kD,
    wheel_kP,
    wheel_kI,
    wheel_kD,
    top_speed,
    max_steering_rate
  );

  // Try building with all params
  PassengerCarDynamicModel pcm;
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

  rosbag::Bag bag;
  bag.open("data/_2019-10-30-11-57-22_5mps_6s.bag", rosbag::bagmode::Read);

  ros::Time startTime(1572454657.9144); // For 5mps 1572454657.9144, for 10mps 1572454805.37492, for 5mps brake 1572454661.71489
  ros::Time endTime(1572454661.71489); // For 5mps ramp 1572454661.71489, for 10mps ramp 1572454813.07975


  std::string imu_tpc         = "/imu_raw";
  std::string wheel_rpt_tpc   = "/pacmod/parsed_tx/wheel_speed_rpt";
  std::string vehicle_cmd_tpc = "/vehicle_cmd";
  std::string gps_tpc         = "/gnss_fix_fused";

  std::vector<std::string> topics;
  topics.push_back(imu_tpc); 
  topics.push_back(wheel_rpt_tpc);
  topics.push_back(vehicle_cmd_tpc);
  topics.push_back(gps_tpc);

  //rosbag::View view(bag, rosbag::TopicQuery(topics), startTime);
  //rosbag::View view(bag, rosbag::TopicQuery(topics), startTime, endTime);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  model_test_tools::BagSubscriber<sensor_msgs::Imu> imu_sub(imu_tpc);
  model_test_tools::BagSubscriber<pacmod_msgs::WheelSpeedRpt> wheel_sub(wheel_rpt_tpc);
  model_test_tools::BagSubscriber<autoware_msgs::VehicleCmd> cmd_sub(vehicle_cmd_tpc);
  model_test_tools::BagSubscriber<gps_common::GPSFix> gps_sub(gps_tpc);

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Imu, pacmod_msgs::WheelSpeedRpt,
    autoware_msgs::VehicleCmd, gps_common::GPSFix
    > ApproxTimePolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), imu_sub, wheel_sub, cmd_sub, gps_sub);
  SpeedStateFunctor functor(speed_test_helper);
  sync.registerCallback(boost::bind<void>(functor, _1, _2, _3, _4));


  //
  // Create Low Pass IIR filter for IMU acceleration data. Based on butterworth filter in matlab
  // Order: 2nd
  // Passband Freq: 21 Hz
  sp::IIR_filt<double,double,double> imu_filter;
  arma::vec3 b;
  b[0] = 0.222955;
  b[1] = 0.445910;
  b[2] = 0.222955;

  arma::vec3 a;
  a[0] = 1.000000;
  a[1] = -0.295200;
  a[2] = 0.187020;

  imu_filter.set_coeffs(b, a);

  // Playback bag file to collect data
  for(const rosbag::MessageInstance m : view)
  {
    // Filter IMU data. This should be done before going into model_test_tools so that the time_synchronizer does not impact the filter
    sensor_msgs::ImuConstPtr s = m.instantiate<sensor_msgs::Imu>();
    sensor_msgs::Imu newMsg;
    if (s != NULL) { //(TODO this requires filtering )
        newMsg.header = s->header;
        newMsg.angular_velocity = s->angular_velocity;
        newMsg.angular_velocity_covariance = s->angular_velocity_covariance;
        newMsg.orientation = s->orientation;
        newMsg.orientation_covariance = s->orientation_covariance;
        newMsg.linear_acceleration = s->linear_acceleration;
        newMsg.linear_acceleration_covariance = s->linear_acceleration_covariance;
        newMsg.linear_acceleration.x = imu_filter(s->linear_acceleration.x);
    }

    sensor_msgs::ImuConstPtr imu_ptr(new sensor_msgs::Imu(newMsg));

    imu_sub.onMessage(imu_ptr);
    wheel_sub.onMessageInstance(m);
    cmd_sub.onMessageInstance(m);
    gps_sub.onMessageInstance(m);
  }

  bag.close();

  std::vector<model_test_tools::SpeedState> bestForcast;
  double minRMSE = 0;
  double bestP = 0;
  double bestI = 0;
  double bestD = 0;
  double minMaxError = 0;
  double minAllowableMaxError = 0.175;
  bool first = true;
  for (double p = 0.0; p < 1.0; p += 0.05) { // P
    if (-0.0000001 < p && p <0.0000001) { // Skip 0
       continue;
    } // Best result so far is 0.7 P with slope limiting to 8.13 and -8.13
    EXPECT_CALL(*mock_param_server, getParam("wheel_kP", A<double&>())).WillRepeatedly(DoAll(set_double(0.7), Return(true)));

    for (double i = -1; i < 1; i += 0.1) { // I
      EXPECT_CALL(*mock_param_server, getParam("wheel_kI", A<double&>())).WillRepeatedly(DoAll(set_double(0.0), Return(true)));

      for (double d = 0.0; d < 3.0; d += 0.1) { // D 
        EXPECT_CALL(*mock_param_server, getParam("wheel_kD", A<double&>())).WillRepeatedly(DoAll(set_double(0.0), Return(true)));

        pcm.setParameterServer(mock_param_server);
        
        std::tuple<double,double> max_error_and_rmse = speed_test_helper.evaluateData(pcm);
        double maxError = std::get<0>(max_error_and_rmse);
        double rmse = std::get<1>(max_error_and_rmse);

        if (first) {
          first = false;
          minRMSE = rmse;
          bestP = p;
          bestI = i;
          bestD = d;
          bestForcast = speed_test_helper.last_forcast_;
          minMaxError = maxError;
        } else if (rmse < minRMSE && (minMaxError > maxError || maxError < minAllowableMaxError)) {
          minRMSE = rmse;
          bestP = p;
          bestI = i;
          bestD = d;
          bestForcast = speed_test_helper.last_forcast_;
          minMaxError = maxError;
        }
        break;
      }
      break;
    }
    std::cerr << "Current P " << p << std::endl;
    std::cerr << "CurBest P: " <<  bestP << " I: " << bestI << " D: " << bestD << std::endl;
    std::cerr << "CurBest RMSE: " << minRMSE << std::endl;
    std::cerr << "CurBest MaxError: " << minMaxError << std::endl;
    break;
  }
  speed_test_helper.logData(bestForcast);
  speed_test_helper.sync_csv_file.close();
  std::cerr << std::endl << "Best P: " <<  bestP << " I: " << bestI << " D: " << bestD << std::endl;
  std::cerr << "Best RMSE: " << minRMSE << std::endl;
  std::cerr << "Best MaxError: " << minMaxError << std::endl;
}


// /**
//  * This Unit test is for finding initial PID values based on bag files.
//  * The determined values should not be taken as guaranteed optimal
//  * Instead they provide a reasonable starting point for tunning. 
//  * 
//  * NOTE: This test is currently disabled. To enable it remove the DISABLED_ prefix from the test name
//  */ 
// TEST(PassengerCarDynamicModel, find_speed_pid)
// {

//   // Create test helper. 
//   // NOTE: Set to true to record data to csv file
//   model_test_tools::SpeedTestHelper speed_test_helper(true); 

//   // Setup param server
//   auto mock_param_server = std::make_shared<MockParamServer>();

//   // Params for this vehicle model
//   // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
//   const double length_to_f = 1.24;
//   const double length_to_r = 1.538;
//   const double unloaded_wheel_radius_f = 0.38354;
//   const double unloaded_wheel_radius_r = 0.38354;
//   const double loaded_wheel_radius_f = 0.355;
//   const double loaded_wheel_radius_r = 0.3625;
//   const double tire_longitudinal_stiffness_f = 90.0; // TODO
//   const double tire_longitudinal_stiffness_r = 90.0; // TODO
//   const double tire_cornering_stiffness_f = 116669.202;
//   const double tire_cornering_stiffness_r = 93993.8288;
//   const double moment_of_inertia = 4859.89;
//   const double vehicle_mass = 2271.5;
//   // NOTE: If steering model is changed then the steering PID must change as well
//   const double steering_kP = 1.0;
//   const double steering_kI = 0.05;
//   const double steering_kD = 0.0;

//   const double wheel_kP = 0.1;
//   const double wheel_kI = 0.0;
//   const double wheel_kD = 0.0;
//   double top_speed = 44.704;
//   double max_steering_rate = 0.333333333;

//   initializeParamServer(
//     mock_param_server,
//     length_to_f,
//     length_to_r,
//     unloaded_wheel_radius_f,
//     unloaded_wheel_radius_r,
//     loaded_wheel_radius_f,
//     loaded_wheel_radius_r,
//     tire_longitudinal_stiffness_f,
//     tire_longitudinal_stiffness_r,
//     tire_cornering_stiffness_f,
//     tire_cornering_stiffness_r,
//     moment_of_inertia,
//     vehicle_mass,
//     steering_kP,
//     steering_kI,
//     steering_kD,
//     wheel_kP,
//     wheel_kI,
//     wheel_kD,
//     top_speed,
//     max_steering_rate
//   );

//   // Try building with all params
//   PassengerCarDynamicModel pcm;
//   ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

//   rosbag::Bag bag;
//   bag.open("data/_2019-10-30-11-57-22_5mps_6s.bag", rosbag::bagmode::Read);

//   ros::Time startTime(1572454661.71489); // For 5mps 1572454657.9144, for 10mps 1572454805.37492, for 5mps brake 1572454661.71489
//   ros::Time endTime(1572454661.71489); // For 5mps ramp 1572454661.71489, for 10mps ramp 1572454813.07975


//   std::string imu_tpc         = "/imu_raw";
//   std::string wheel_rpt_tpc   = "/pacmod/parsed_tx/wheel_speed_rpt";
//   std::string vehicle_cmd_tpc = "/vehicle_cmd";
//   std::string gps_tpc         = "/gnss_fix_fused";

//   std::vector<std::string> topics;
//   topics.push_back(imu_tpc); 
//   topics.push_back(wheel_rpt_tpc);
//   topics.push_back(vehicle_cmd_tpc);
//   topics.push_back(gps_tpc);

//   rosbag::View view(bag, rosbag::TopicQuery(topics), startTime);
//   //rosbag::View view(bag, rosbag::TopicQuery(topics), startTime, endTime);
//   //rosbag::View view(bag, rosbag::TopicQuery(topics));

//   model_test_tools::BagSubscriber<sensor_msgs::Imu> imu_sub(imu_tpc);
//   model_test_tools::BagSubscriber<pacmod_msgs::WheelSpeedRpt> wheel_sub(wheel_rpt_tpc);
//   model_test_tools::BagSubscriber<autoware_msgs::VehicleCmd> cmd_sub(vehicle_cmd_tpc);
//   model_test_tools::BagSubscriber<gps_common::GPSFix> gps_sub(gps_tpc);

//   typedef message_filters::sync_policies::ApproximateTime<
//     sensor_msgs::Imu, pacmod_msgs::WheelSpeedRpt,
//     autoware_msgs::VehicleCmd, gps_common::GPSFix
//     > ApproxTimePolicy;

//   // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//   message_filters::Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), imu_sub, wheel_sub, cmd_sub, gps_sub);
//   SpeedStateFunctor functor(speed_test_helper);
//   sync.registerCallback(boost::bind<void>(functor, _1, _2, _3, _4));


//   //
//   // Create Low Pass IIR filter for IMU acceleration data. Based on butterworth filter in matlab
//   // Order: 2nd
//   // Passband Freq: 21 Hz
//   sp::IIR_filt<double,double,double> imu_filter;
//   arma::vec3 b;
//   b[0] = 0.222955;
//   b[1] = 0.445910;
//   b[2] = 0.222955;

//   arma::vec3 a;
//   a[0] = 1.000000;
//   a[1] = -0.295200;
//   a[2] = 0.187020;

//   imu_filter.set_coeffs(b, a);

//   // Playback bag file to collect data
//   for(const rosbag::MessageInstance m : view)
//   {
//     // Filter IMU data. This should be done before going into model_test_tools so that the time_synchronizer does not impact the filter
//     sensor_msgs::ImuConstPtr s = m.instantiate<sensor_msgs::Imu>();
//     sensor_msgs::Imu newMsg;
//     if (s != NULL) { //(TODO this requires filtering )
//         newMsg.header = s->header;
//         newMsg.angular_velocity = s->angular_velocity;
//         newMsg.angular_velocity_covariance = s->angular_velocity_covariance;
//         newMsg.orientation = s->orientation;
//         newMsg.orientation_covariance = s->orientation_covariance;
//         newMsg.linear_acceleration = s->linear_acceleration;
//         newMsg.linear_acceleration_covariance = s->linear_acceleration_covariance;
//         newMsg.linear_acceleration.x = imu_filter(s->linear_acceleration.x);
//     }

//     sensor_msgs::ImuConstPtr imu_ptr(new sensor_msgs::Imu(newMsg));

//     imu_sub.onMessage(imu_ptr);
//     wheel_sub.onMessageInstance(m);
//     cmd_sub.onMessageInstance(m);
//     gps_sub.onMessageInstance(m);
//   }

//   bag.close();

//   std::vector<model_test_tools::SpeedState> bestForcast;
//   double minRMSE = 0;
//   double bestP = 0;
//   double bestI = 0;
//   double bestD = 0;
//   double minMaxError = 0;
//   double minAllowableMaxError = 0.175;
//   bool first = true;
//   // Best result so far is 0.7 P with slope limiting to 8.13 and -8.13
//     EXPECT_CALL(*mock_param_server, getParam("wheel_kP", A<double&>())).WillRepeatedly(DoAll(set_double(0.7), Return(true)));
//     EXPECT_CALL(*mock_param_server, getParam("wheel_kI", A<double&>())).WillRepeatedly(DoAll(set_double(0.0), Return(true)));
//     EXPECT_CALL(*mock_param_server, getParam("wheel_kD", A<double&>())).WillRepeatedly(DoAll(set_double(0.0), Return(true)));

//     pcm.setParameterServer(mock_param_server);
      
//     // std::tuple<double,double> max_error_and_rmse = speed_test_helper.evaluateData(pcm);
//     // double maxError = std::get<0>(max_error_and_rmse);
//     // double rmse = std::get<1>(max_error_and_rmse);

// }

