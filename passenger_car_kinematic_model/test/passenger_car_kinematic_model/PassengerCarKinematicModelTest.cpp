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
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <pacmod_msgs/WheelSpeedRpt.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <automotive_platform_msgs/SteeringFeedback.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "lib_vehicle_model/VehicleControlInput.h"
#include "lib_vehicle_model/VehicleState.h"
#include "lib_vehicle_model/ParameterServer.h"
#include "lib_vehicle_model/LibVehicleModel.h"
#include "lib_vehicle_model/ODESolver.h"
#include "passenger_car_kinematic_model/PassengerCarKinematicModel.h"
#include <model_test_tools/TestHelper.h>


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

class ParameterInitializer {
  public:
    // Default values from STOL Lexus
    double length_to_f_ = 1.24;
    double length_to_r_ = 1.538;
    double unloaded_wheel_radius_f_ = 0.355;
    double unloaded_wheel_radius_r_ = 0.3625;
    double loaded_wheel_radius_f_ = 0.355;
    double loaded_wheel_radius_r_ = 0.3625;
    double speed_kP_ = 0.7;


  void initializeParamServer (
    std::shared_ptr<MockParamServer> mock_param_server
  ) {
    EXPECT_CALL(*mock_param_server, getParam("length_to_f", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_f_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("length_to_r", A<double&>())).WillRepeatedly(DoAll(set_double(length_to_r_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("unloaded_wheel_radius_f", A<double&>())).WillRepeatedly(DoAll(set_double(unloaded_wheel_radius_f_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("unloaded_wheel_radius_r", A<double&>())).WillRepeatedly(DoAll(set_double(unloaded_wheel_radius_r_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("loaded_wheel_radius_f", A<double&>())).WillRepeatedly(DoAll(set_double(loaded_wheel_radius_f_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("loaded_wheel_radius_r", A<double&>())).WillRepeatedly(DoAll(set_double(loaded_wheel_radius_r_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("speed_kP", A<double&>())).WillRepeatedly(DoAll(set_double(speed_kP_), Return(true)));
  }
};



/**
 * Tests the setParameterServer function of the PassengerCarKinematicModel class
 */ 
TEST(PassengerCarKinematicModel, setParameterServer)
{
  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model but with mass set to false
  ParameterInitializer paramIniter;
  paramIniter.initializeParamServer(mock_param_server);
  // Make one param return false
  EXPECT_CALL(*mock_param_server, getParam("speed_kP", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(false)));
  
  // Try building without all params
  PassengerCarKinematicModel pcm;

  ASSERT_THROW(pcm.setParameterServer(mock_param_server), std::invalid_argument);

  // Add mass to params
  EXPECT_CALL(*mock_param_server, getParam("speed_kP", A<double&>())).WillRepeatedly(DoAll(set_double(-180.0), Return(true)));

  // Try loading valid set of parameters
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));
}


// /**
//  * Tests the predict (no control input) function of the PassengerCarKinematicModel 
//  */ 
// TEST(lib_vehicle_model, predict_no_control)
// {
//     // Setup param server
//   auto mock_param_server = std::make_shared<MockParamServer>();

//   // Params for this vehicle model but with mass set to false
//   const double wheel_radius = 0.3048; //m
//   const double wheel_base = 2.4384 * 2.0;
//   initializeParamServer(mock_param_server, wheel_base, wheel_radius, wheel_radius);
  
//   // Try building without all params
//   PassengerCarKinematicModel pcm;

//   // Try loading valid set of parameters
//   ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

//   // Test no slip condition for 1 timestep
//   lib_vehicle_model::VehicleState vs;
//   vs.X_pos_global = 0;
//   vs.Y_pos_global = 0;
//   vs.orientation = 0;
//   vs.longitudinal_vel = 5;
//   vs.lateral_vel = 0;
//   vs.yaw_rate = 0;
//   vs.front_wheel_rotation_rate = vs.longitudinal_vel / wheel_radius;
//   vs.rear_wheel_rotation_rate = vs.front_wheel_rotation_rate;
//   vs.steering_angle = 0;
//   vs.trailer_angle = 0;
//   vs.prev_steering_cmd = 0;
//   vs.prev_vel_cmd = vs.longitudinal_vel;

//   // Build debug string for initial error
//   std::vector<lib_vehicle_model::VehicleState> result = pcm.predict(vs,0.1,0.1);
//   std::ostringstream msg;
//   msg << "Size: " << result.size() << std::endl;;
//   for (lib_vehicle_model::VehicleState v : result) {
//     msg << v << std::endl;
//   }

//   // Check result size
//   ASSERT_EQ(1, result.size());
//   for (auto v: result) {
//     ASSERT_NEAR(0.5, v.X_pos_global, 0.0000001) << msg.str();
//     ASSERT_NEAR(vs.Y_pos_global, v.Y_pos_global, 0.0000001);
//     ASSERT_NEAR(vs.orientation, v.orientation, 0.0000001);
//     ASSERT_NEAR(vs.longitudinal_vel, v.longitudinal_vel, 0.0000001);
//     ASSERT_NEAR(vs.lateral_vel, v.lateral_vel, 0.0000001);
//     ASSERT_NEAR(vs.yaw_rate, v.yaw_rate, 0.0000001);
//     ASSERT_NEAR(vs.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs.steering_angle, v.steering_angle, 0.0000001);
//     ASSERT_NEAR(vs.trailer_angle, v.trailer_angle, 0.0000001);
//     ASSERT_NEAR(vs.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
//     ASSERT_NEAR(vs.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
//   }


//   // Test no slip condition for 5 timesteps
//   result = pcm.predict(vs,0.1,0.5);
//   // Check result size
//   ASSERT_EQ(5, result.size());
//   int i = 1;
//   for (auto v: result) {
//     ASSERT_NEAR(0.1 * i * vs.longitudinal_vel, v.X_pos_global, 0.0000001);
//     ASSERT_NEAR(vs.Y_pos_global, v.Y_pos_global, 0.0000001);
//     ASSERT_NEAR(vs.orientation, v.orientation, 0.0000001);
//     ASSERT_NEAR(vs.longitudinal_vel, v.longitudinal_vel, 0.0000001);
//     ASSERT_NEAR(vs.lateral_vel, v.lateral_vel, 0.0000001);
//     ASSERT_NEAR(vs.yaw_rate, v.yaw_rate, 0.0000001);
//     ASSERT_NEAR(vs.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs.steering_angle, v.steering_angle, 0.0000001);
//     ASSERT_NEAR(vs.trailer_angle, v.trailer_angle, 0.0000001);
//     ASSERT_NEAR(vs.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
//     ASSERT_NEAR(vs.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
//     i++;
//   }

//   // Test no slip condition for 5 timesteps with different orientation
//   vs.orientation = M_PI_2; // Change orientation to be facing directly along y axis
//   result = pcm.predict(vs,0.1,0.5);
//   // Check result size
//   ASSERT_EQ(5, result.size());
//   i = 1;
//   for (auto v: result) {
//     ASSERT_NEAR(vs.X_pos_global, v.X_pos_global, 0.0000001);
//     ASSERT_NEAR(0.1 * i * vs.longitudinal_vel, v.Y_pos_global, 0.0000001);
//     ASSERT_NEAR(vs.orientation, v.orientation, 0.0000001);
//     ASSERT_NEAR(vs.longitudinal_vel, v.longitudinal_vel, 0.0000001);
//     ASSERT_NEAR(vs.lateral_vel, v.lateral_vel, 0.0000001);
//     ASSERT_NEAR(vs.yaw_rate, v.yaw_rate, 0.0000001);
//     ASSERT_NEAR(vs.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs.steering_angle, v.steering_angle, 0.0000001);
//     ASSERT_NEAR(vs.trailer_angle, v.trailer_angle, 0.0000001);
//     ASSERT_NEAR(vs.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
//     ASSERT_NEAR(vs.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
//     i++;
//   }
  

//   // Try stopped vehicle
//   lib_vehicle_model::VehicleState vs_stop;
//   result = pcm.predict(vs_stop,0.1,0.5);
//   // Check result size
//   ASSERT_EQ(5, result.size());
//   i = 1;
//   for (auto v: result) {
//     ASSERT_NEAR(vs_stop.X_pos_global, v.X_pos_global, 0.0000001);
//     ASSERT_NEAR(vs_stop.Y_pos_global, v.Y_pos_global, 0.0000001);
//     ASSERT_NEAR(vs_stop.orientation, v.orientation, 0.0000001);
//     ASSERT_NEAR(vs_stop.longitudinal_vel, v.longitudinal_vel, 0.0000001);
//     ASSERT_NEAR(vs_stop.lateral_vel, v.lateral_vel, 0.0000001);
//     ASSERT_NEAR(vs_stop.yaw_rate, v.yaw_rate, 0.0000001);
//     ASSERT_NEAR(vs_stop.front_wheel_rotation_rate, v.front_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs_stop.rear_wheel_rotation_rate, v.rear_wheel_rotation_rate, 0.0000001);
//     ASSERT_NEAR(vs_stop.steering_angle, v.steering_angle, 0.0000001);
//     ASSERT_NEAR(vs_stop.trailer_angle, v.trailer_angle, 0.0000001);
//     ASSERT_NEAR(vs_stop.prev_steering_cmd, v.prev_steering_cmd, 0.0000001);
//     ASSERT_NEAR(vs_stop.prev_vel_cmd, v.prev_vel_cmd, 0.0000001);
//     i++;
//   }
// }

// class VehicleStateFunctor {
//   model_test_tools::ModelTestHelper& helper_;
//   public:
//     VehicleStateFunctor(model_test_tools::ModelTestHelper& helper) : helper_(helper)
//     {}

//     // Callback for ode observer during integration
//     void operator()(const geometry_msgs::PoseStampedConstPtr pose_msg,
//     const sensor_msgs::ImuConstPtr imu_msg,
//     const pacmod_msgs::WheelSpeedRptConstPtr wheel_msg,
//     const geometry_msgs::TwistStampedConstPtr twist_msg,
//     const autoware_msgs::VehicleCmdConstPtr cmd_msg,
//     const automotive_platform_msgs::SteeringFeedbackConstPtr steer_msg)
//     {
//       const double radPerSteeringRad = 0.05922;
//       double stamp = pose_msg->header.stamp.toSec();
//       lib_vehicle_model::VehicleState vs;
//       vs.X_pos_global = pose_msg->pose.position.x;
//       vs.Y_pos_global = pose_msg->pose.position.y;
//       double roll, pitch, yaw;
//       model_test_tools::getRPYFromPose(pose_msg->pose, roll, pitch, yaw);

//       vs.orientation = yaw;
//       vs.longitudinal_vel = twist_msg->twist.linear.x;
//       vs.lateral_vel = 0;

//       vs.yaw_rate = imu_msg->angular_velocity.z;
//       vs.front_wheel_rotation_rate = (wheel_msg->front_left_wheel_speed + wheel_msg->front_right_wheel_speed) / 2.0; // Front wheel speed average
//       vs.rear_wheel_rotation_rate = (wheel_msg->rear_left_wheel_speed + wheel_msg->rear_right_wheel_speed) / 2.0; // Front wheel speed average
//       vs.steering_angle = steer_msg->steering_wheel_angle * radPerSteeringRad; // Convert steering wheel angle to wheel angle

//       lib_vehicle_model::VehicleControlInput ctrl_input;
//       ctrl_input.target_steering_angle = cmd_msg->ctrl_cmd.steering_angle;
//       ctrl_input.target_velocity = cmd_msg->ctrl_cmd.linear_velocity;

//       helper_.vehicleStateCallback(stamp, vs, ctrl_input); 
//     }
// };

// /**
//  * Tests the overall prediction performance of the model
//  * 
//  * NOTE: To view the data from this unit test set the steer_test_helper(true) for one rosbag at a time and look at the steering_sync.csv file
//  */ 
// TEST(PassengerCarKinematicModel, evaluate_overall_pred)
// {

//   // Setup param server
//   auto mock_param_server = std::make_shared<MockParamServer>();

//   // Params for this vehicle model
//   // NOTE: The values here are from the measurements of the STOL Blue Lexus in Sep. 2019
//   const double wheel_base = 1.24 + 1.538;
//   const double effective_wheel_radius_f = 0.355;
//   const double effective_wheel_radius_r = 0.3625;

//   initializeParamServer(
//     mock_param_server,
//     wheel_base,
//     effective_wheel_radius_f,
//     effective_wheel_radius_r
//   );

//   // Try building with all params
//   PassengerCarKinematicModel pcm;
//   ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

//   //// 
//   // Open Data File
//   ////
//   model_test_tools::ModelTestHelper model_test_helper(true, 0.1, 6.0);

//   rosbag::Bag bag;
//   bag.open("data/_2019-10-03-17-11-49_full_loop_validation_filtered.bag", rosbag::bagmode::Read);
  
//   std::string imu_tpc               = "/hardware_interface/imu_raw";
//   std::string wheel_rpt_tpc         = "/hardware_interface/pacmod/parsed_tx/wheel_speed_rpt";
//   std::string steering_feedback_tpc = "/hardware_interface/steering_feedback";
//   std::string twist_tpc             = "/hardware_interface/vehicle/twist";
//   std::string vehicle_cmd_tpc       = "/hardware_interface/vehicle_cmd";
//   std::string pose_tpc              = "/localization/ndt_pose";

//   std::vector<std::string> topics;
//   topics.push_back(imu_tpc); 
//   topics.push_back(wheel_rpt_tpc);
//   topics.push_back(steering_feedback_tpc);
//   topics.push_back(twist_tpc);
//   topics.push_back(vehicle_cmd_tpc);
//   topics.push_back(pose_tpc);

//   rosbag::View view(bag, rosbag::TopicQuery(topics));

//   model_test_tools::BagSubscriber<sensor_msgs::Imu> imu_sub(imu_tpc);
//   model_test_tools::BagSubscriber<pacmod_msgs::WheelSpeedRpt> wheel_sub(wheel_rpt_tpc);
//   model_test_tools::BagSubscriber<automotive_platform_msgs::SteeringFeedback> steer_sub(steering_feedback_tpc);
//   model_test_tools::BagSubscriber<geometry_msgs::TwistStamped> twist_sub(twist_tpc);
//   model_test_tools::BagSubscriber<autoware_msgs::VehicleCmd> cmd_sub(vehicle_cmd_tpc);
//   model_test_tools::BagSubscriber<geometry_msgs::PoseStamped> pose_sub(pose_tpc);

//   typedef message_filters::sync_policies::ApproximateTime<
//     geometry_msgs::PoseStamped, sensor_msgs::Imu, pacmod_msgs::WheelSpeedRpt,
//     geometry_msgs::TwistStamped, autoware_msgs::VehicleCmd, automotive_platform_msgs::SteeringFeedback
//     > ApproxTimePolicy;

//   // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//   message_filters::Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), pose_sub, imu_sub, wheel_sub, twist_sub, cmd_sub, steer_sub);

//   VehicleStateFunctor cb_functor(model_test_helper);
//   sync.registerCallback(boost::bind<void>(cb_functor, _1, _2, _3, _4, _5, _6));

//   // Playback bag file to collect data
//   for(const rosbag::MessageInstance m : view)
//   {
//     pose_sub.onMessageInstance(m);
//     imu_sub.onMessageInstance(m);
//     wheel_sub.onMessageInstance(m);
//     twist_sub.onMessageInstance(m);
//     cmd_sub.onMessageInstance(m);
//     steer_sub.onMessageInstance(m);
//   }

//   bag.close();

//   //std::tuple<double,double> max_error_and_rmse = steer_test_helper.evaluateSteeringData(pcm);
//   model_test_helper.evaluateData(pcm);
//   // double maxError = std::get<0>(max_error_and_rmse);
//   // double rmse = std::get<1>(max_error_and_rmse);

//   // ASSERT_LE(rmse, 0.03) << " RMSE larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
//   // ASSERT_LE(maxError, 0.12015) << " MaxError larger then allowed " << "PID - P: " << steering_kP << " I: " << steering_kI << " D: " << steering_kD;
  
//   model_test_helper.logData(model_test_helper.last_forcast_);
//   model_test_helper.sync_csv_file.close();
// }
