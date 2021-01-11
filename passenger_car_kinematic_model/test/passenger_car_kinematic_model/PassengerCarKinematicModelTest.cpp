/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <gps_common/GPSFix.h>
#include <sigpack/sigpack.h>


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
    double speed_kP_ = 0.8;
    double acceleration_limit_ = 3.0;
    double deceleration_limit_ = 6.0;
    double hard_braking_threshold_ = 2.2;

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
    EXPECT_CALL(*mock_param_server, getParam("acceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(acceleration_limit_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("deceleration_limit", A<double&>())).WillRepeatedly(DoAll(set_double(deceleration_limit_), Return(true)));
    EXPECT_CALL(*mock_param_server, getParam("hard_braking_threshold", A<double&>())).WillRepeatedly(DoAll(set_double(hard_braking_threshold_), Return(true)));
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


/**
 * Tests the predict (no control input) function of the PassengerCarKinematicModel 
 */ 
TEST(lib_vehicle_model, predict_no_control)
{
    // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  const double wheel_radius = 0.3048; //m
  // Params for this vehicle model
  ParameterInitializer paramIniter;
  paramIniter.length_to_f_             = 2.4384;
  paramIniter.length_to_r_             = 2.4384;
  paramIniter.unloaded_wheel_radius_f_ = wheel_radius;
  paramIniter.unloaded_wheel_radius_r_ = wheel_radius;
  paramIniter.loaded_wheel_radius_f_   = wheel_radius;
  paramIniter.loaded_wheel_radius_r_   = wheel_radius;
  paramIniter.initializeParamServer(mock_param_server);

  
  // Try building without all params
  PassengerCarKinematicModel pcm;

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

class VehicleStateFunctor {
  model_test_tools::ModelTestHelper& helper_;
  double base_link_to_CG_dist_ = 0;
  public:
    VehicleStateFunctor(model_test_tools::ModelTestHelper& helper, double base_link_to_CG_dist) : helper_(helper), base_link_to_CG_dist_(base_link_to_CG_dist)
    {}

    // Callback for ode observer during integration
    void operator()(const geometry_msgs::PoseStampedConstPtr pose_msg,
    const sensor_msgs::ImuConstPtr imu_msg,
    const pacmod_msgs::WheelSpeedRptConstPtr wheel_msg,
    const geometry_msgs::TwistStampedConstPtr twist_msg,
    const autoware_msgs::VehicleCmdConstPtr cmd_msg,
    const automotive_platform_msgs::SteeringFeedbackConstPtr steer_msg)
    {
      const double radPerSteeringRad = 0.05922;
      double stamp = pose_msg->header.stamp.toSec();
      lib_vehicle_model::VehicleState vs;
      double roll, pitch, yaw;
      model_test_tools::getRPYFromPose(pose_msg->pose, roll, pitch, yaw);

      vs.orientation = yaw;

      // Pose is provided in the base_link frame but the position of the CG is needed for this model
      // Add the components of the offset from base_link to cg to the model
      vs.X_pos_global = pose_msg->pose.position.x + (base_link_to_CG_dist_ * cos(yaw));
      vs.Y_pos_global = pose_msg->pose.position.y + (base_link_to_CG_dist_ * sin(yaw));

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
 * Tests the overall prediction performance of the model
 * 
 * NOTE: To view the data from this unit test set the steer_test_helper(true) for one rosbag at a time and look at the steering_sync.csv file
 */ 
TEST(PassengerCarKinematicModel, evaluate_overall_pred)
{

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

  // Params for this vehicle model
  ParameterInitializer paramIniter;
  paramIniter.initializeParamServer(mock_param_server);

  // Try building with all params
  PassengerCarKinematicModel pcm;
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

  //// 
  // Open Data File
  ////
  model_test_tools::ModelTestHelper model_test_helper(false, 0.1, 6.0);

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

  VehicleStateFunctor cb_functor(model_test_helper, paramIniter.length_to_r_);
  sync.registerCallback(boost::bind<void>(cb_functor, _1, _2, _3, _4, _5, _6));

  // Playback bag file to collect data
  for(const rosbag::MessageInstance m : view)
  {
    pose_sub.onMessageInstance(m);
    imu_sub.onMessageInstance(m);
    wheel_sub.onMessageInstance(m);
    twist_sub.onMessageInstance(m);
    cmd_sub.onMessageInstance(m);
    steer_sub.onMessageInstance(m);
  }

  bag.close();

  model_test_helper.evaluateData(pcm);
  
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
    ss.longitudinal_accel = imu_msg->linear_acceleration.y; // NOTE: this changes to x or y depending on car
    ss.front_wheel_angular_vel = (wheel_msg->front_left_wheel_speed + wheel_msg->front_right_wheel_speed) / 2.0;
    ss.rear_wheel_angular_vel = (wheel_msg->rear_left_wheel_speed + wheel_msg->rear_right_wheel_speed) / 2.0;
    ss.longitudinal_vel = fix_msg->speed; // NOTE: This assumes forward driving

    // Store data
    helper_.syncCallback(stamp, ss, cmd_vel);
  }
};


/**
 * This Unit test is for finding initial P values based on bag files.
 * The determined values should not be taken as guaranteed optimal
 * Instead they provide a reasonable starting point for tunning. 
 * 
 * NOTE: This test is currently disabled. To enable it remove the DISABLED_ prefix from the test name
 */ 
TEST(PassengerCarKinematicModel, DISABLED_find_speed_pid)
{

  // Create test helper. 
  // NOTE: Set to true to record data to csv file
  model_test_tools::SpeedTestHelper speed_test_helper(true); 

  // Setup param server
  auto mock_param_server = std::make_shared<MockParamServer>();

    // Params for this vehicle model
  ParameterInitializer paramIniter;
  paramIniter.acceleration_limit_ = 2.0;
  paramIniter.deceleration_limit_ = 5.0;
  paramIniter.initializeParamServer(mock_param_server);
  // Try building with all params
  PassengerCarKinematicModel pcm;
  ASSERT_NO_THROW(pcm.setParameterServer(mock_param_server));

  rosbag::Bag bag;
  bag.open("data/_2019-10-30-11-57-22_5mps_6s_filtered.bag", rosbag::bagmode::Read);

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
  rosbag::View view(bag, rosbag::TopicQuery(topics), startTime, endTime);
  //rosbag::View view(bag, rosbag::TopicQuery(topics));

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
    if (s != NULL) { 
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
  bool first = true;
  for (double p = 0.0; p < 1.5; p += 0.05) { // P
    if (-0.0000001 < p && p <0.0000001) { // Skip 0
       continue;
    } // Best result so far is 0.8 P
    EXPECT_CALL(*mock_param_server, getParam("speed_kP", A<double&>())).WillRepeatedly(DoAll(set_double(p), Return(true)));

    pcm.setParameterServer(mock_param_server);
    
    std::tuple<double,double> max_error_and_rmse = speed_test_helper.evaluateData(pcm);
    double maxError = std::get<0>(max_error_and_rmse);
    double rmse = std::get<1>(max_error_and_rmse);

    if (first) {
      first = false;
      minRMSE = rmse;
      bestP = p;
      bestForcast = speed_test_helper.last_forcast_;
      minMaxError = maxError;
    } else if (rmse < minRMSE) {
      minRMSE = rmse;
      bestP = p;
      bestForcast = speed_test_helper.last_forcast_;
      minMaxError = maxError;
    }
    std::cerr << "Current P " << p << std::endl;
    std::cerr << "CurBest P: " <<  bestP << std::endl;
    std::cerr << "CurBest RMSE: " << minRMSE << std::endl;
    std::cerr << "CurBest MaxError: " << minMaxError << std::endl;
    //break;
  }
  speed_test_helper.logData(bestForcast);
  speed_test_helper.sync_csv_file.close();
  std::cerr << std::endl << "Best P: " <<  bestP << std::endl;
  std::cerr << "Best RMSE: " << minRMSE << std::endl;
  std::cerr << "Best MaxError: " << minMaxError << std::endl;
}