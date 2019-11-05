#pragma once

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

#include <message_filters/subscriber.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/shared_ptr.hpp>
#include <autoware_msgs/VehicleCmd.h>
#include <automotive_platform_msgs/SteeringFeedback.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include "lib_vehicle_model/VehicleControlInput.h"
#include "lib_vehicle_model/VehicleState.h"
#include "lib_vehicle_model/VehicleMotionModel.h"

namespace model_test_tools {
  /**
   * \brief Class Based on Section 2.1 in ROS Bag cookbook http://wiki.ros.org/rosbag/Cookbook
   * Inherits from message_filters::SimpleFilter<M>
   * to use protected signalMessage function 
   */
  template <class M>
  class BagSubscriber : public message_filters::SimpleFilter<M>
  {
    private:
      std::string topic_;
      typedef boost::shared_ptr<M const> MConstPtr;
    public:
      
      BagSubscriber(const std::string& topic) : topic_(topic) {}
      
      void onMessageInstance(const rosbag::MessageInstance& m) {
        if (m.getTopic() == topic_ || ("/" + m.getTopic() == topic_))
        {
          MConstPtr msg = m.instantiate<M>();
          if (msg != NULL)
            this->signalMessage(msg);
        }
      }

      void onMessage(const MConstPtr& m) {
        this->signalMessage(m);
      }
  };

  /**
   * \brief Helper function to compute the max error and RMSE of forcasts vs observed data
   * returns a tuple where the first value is the max error and the second is the rmse
   * 
   * \param forcast List of predicted values
   * \param observed List of observed values. Must have the same length as forcast
   * 
   * \return A tuple of max error and RMSE where tuple element 0 is the max error and tuple element 1 is the RMSE value
   */ 
  std::tuple<double,double> getMaxErrorAndRMSE(const std::vector<double>& forcast, const std::vector<double>& observed) {
    //std::cerr << "Forcast size: " << forcast.size() << " Observed Size: " << observed.size() << std::endl;
    double sumSqrs = 0;
    double maxError = 0;
    for (size_t i = 0; i < forcast.size(); i++) {
      double vf = forcast[i];
      double vo = observed[i];

      // Find max error
      double abs_error = fabs(vf - vo);
      if (maxError < abs_error) {
        maxError = abs_error;
      }

      // Update sum squares
      double diff = vf - vo;
      double diff_sqr = diff * diff;
      sumSqrs += diff_sqr;
    }

    double rmse = sqrt(sumSqrs / forcast.size());
    return std::make_tuple(maxError, rmse);
  }

  /**
   * \brief Helper function to extract RPY from a pose message
   * 
   * \param pose The pose message containing an orientation
   * \param roll Variable to store roll value in rad
   * \param pitch Variable to store pitch value in rad
   * \param yaw Variable to store yaw value in rad
   */ 
  void getRPYFromPose(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw) {
    tf2::Quaternion quat (
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    );

    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
  }

  /**
   * \brief Class to assist in unit testing of vehicle models
   * 
   * User passes in recorded data using the vehicleStateCallback function
   * This data is stored and will be used when the user calls the evaluateData function using the vehicle model to be tested
   * If desired this class can log results to a file using the logData function.
   * THe last_forcast_ object can be used to access the most resent results of evaluateData 
   */ 
  class ModelTestHelper {
    public: 
      const bool write_to_file_ = false;
      const double timestep_ = 0.0;
      const double pred_period_ = 0.0;

      ModelTestHelper(bool write_to_file, double timestep, double pred_period) : write_to_file_(write_to_file), timestep_(timestep), pred_period_(pred_period) {}

      ~ModelTestHelper() {
        sync_csv_file.close();
      }

      std::vector<lib_vehicle_model::VehicleState> current_states_, last_forcast_;
      std::vector<lib_vehicle_model::VehicleControlInput> current_cmds_;
      std::vector<double> stamps;

      std::ofstream sync_csv_file;

      void vehicleStateCallback(double stamp, lib_vehicle_model::VehicleState vs, lib_vehicle_model::VehicleControlInput vc) {

        if (current_cmds_.size() > 0) {
          vs.prev_vel_cmd = current_cmds_[current_cmds_.size() - 1].target_velocity;
          vs.prev_steering_cmd = current_cmds_[current_cmds_.size() - 1].target_steering_angle;
        }

        current_states_.push_back(vs);
        current_cmds_.push_back(vc);
        stamps.push_back(stamp);

      }

      void logData(std::vector<lib_vehicle_model::VehicleState>& pred) {
        if (!write_to_file_)
          return;
        
        if (!sync_csv_file.is_open()) {
          sync_csv_file.open("veh_pred_sync.csv");
          sync_csv_file << "Stamp (s), Cmd Steer (rad), Cmd Vel (rad),"
            << " Actual X (m), Actual Y (m), Actual Yaw (m)," 
            << " Actual lon-vel (m/s), Actual lat-vel (m/s), Actual yaw-rate (rad/s),"
            << " Actual f-wheel rate (rad/s), Actual r-wheel rate (rad/s), Actual steer angle (rad),"

            << " Pred X (m), Pred Y (m), Pred Yaw (m)," 
            << " Pred lon-vel (m/s), Pred lat-vel (m/s), Pred yaw-rate (rad/s),"
            << " Pred f-wheel rate (rad/s), Pred r-wheel rate (rad/s), Pred steer angle (rad)," 
            
            << " Pred Err X (m), Pred Err Y (m), Pred Err Yaw (rad)," 
            << " Pred Err lon-vel (m/s), Pred Err lat-vel (m/s), Pred Err yaw-rate (rad/s),"
            << " Pred Err f-wheel rate (rad/s), Pred Err r-wheel rate (rad/s), Pred Err steer angle (rad)," 
            << std::endl;
          sync_csv_file << std::setprecision(20);
        }

        for (size_t n = 0; n < stamps.size(); n++) {
          
          sync_csv_file << stamps[n] - stamps[0] << "," << current_cmds_[n].target_steering_angle << "," << current_cmds_[n].target_velocity << ", "
            << current_states_[n].X_pos_global << "," << current_states_[n].Y_pos_global << "," << current_states_[n].orientation << ", "
            << current_states_[n].longitudinal_vel << "," << current_states_[n].lateral_vel << "," << current_states_[n].yaw_rate << ", "
            << current_states_[n].front_wheel_rotation_rate << "," << current_states_[n].rear_wheel_rotation_rate << "," << current_states_[n].steering_angle << ", "
            
            << pred[n].X_pos_global << "," << pred[n].Y_pos_global << "," << pred[n].orientation << ", "
            << pred[n].longitudinal_vel << "," << pred[n].lateral_vel << "," << pred[n].yaw_rate << ", "
            << pred[n].front_wheel_rotation_rate << "," << pred[n].rear_wheel_rotation_rate << "," << pred[n].steering_angle << ", "

            << current_states_[n].X_pos_global - pred[n].X_pos_global << "," << current_states_[n].Y_pos_global - pred[n].Y_pos_global << "," << current_states_[n].orientation - pred[n].orientation << ", "
            << current_states_[n].longitudinal_vel - pred[n].longitudinal_vel << "," << current_states_[n].lateral_vel - pred[n].lateral_vel << "," << current_states_[n].yaw_rate - pred[n].yaw_rate << ", "
            << current_states_[n].front_wheel_rotation_rate - pred[n].front_wheel_rotation_rate << "," << current_states_[n].rear_wheel_rotation_rate - pred[n].rear_wheel_rotation_rate << "," << current_states_[n].steering_angle - pred[n].steering_angle
            << std::endl;
        }
      }

      void evaluateData(lib_vehicle_model::VehicleMotionModel& pcm) {
        std::vector<lib_vehicle_model::VehicleState> forcasts;
        forcasts.reserve(stamps.size());
        double timestep = timestep_;
        double max_pred_time = pred_period_;
     
        double prev_pred_end_time = 0; 
        size_t pred_start_idx = 0;
        bool firstLoop = true;
        for (size_t n = 0; n < stamps.size(); n++) {
          double time = stamps[n];
          if (firstLoop) {
            prev_pred_end_time = time;
            pred_start_idx = n;
            firstLoop = false;
          } else if (time - prev_pred_end_time > max_pred_time || n == stamps.size() - 1) { // If max_pred_time has passed since the last prediction period
            // Predict vehicle motion over just the last pred period
            std::vector<lib_vehicle_model::VehicleControlInput> control_inputs_section(current_cmds_.begin() + pred_start_idx, current_cmds_.begin() + n);
            // std::cerr << "Control Input Section Start at Index: " << pred_start_idx << " Ending at: " << n << std::endl;
            // for (auto control_input : control_inputs_section) {
            //   std::cerr << "Cmd Speed: " << control_input.target_velocity << " Cmd Steer: " << control_input.target_steering_angle << std::endl;
            // }
            // std::cerr << std::endl;
            
            auto subForcast = pcm.predict(current_states_[pred_start_idx], control_inputs_section, timestep);
            
             std::cerr << std::endl;
            
            forcasts.insert( forcasts.end(), subForcast.begin(), subForcast.end() ); // Add prediction for this setpoint to forcasts list
            prev_pred_end_time = time;
            pred_start_idx = n;
          }
        }

        last_forcast_ = forcasts;

        //TODO we probably want to still compute RMSE and error for each pred return test_helper::getMaxErrorAndRMSE(forcasts,current_steers);
      }
    };


  struct SpeedState {
    double front_wheel_angular_vel = 0;
    double rear_wheel_angular_vel = 0;
    double longitudinal_vel = 0;
    double longitudinal_accel = 0;
  };

  class SpeedTestHelper {
    public: 
      const bool write_to_file_ = false;

      SpeedTestHelper(bool write_to_file) : write_to_file_(write_to_file) {}

      ~SpeedTestHelper() {
        sync_csv_file.close();
      }


      std::vector<SpeedState> current_states_, last_forcast_;
      std::vector<double> current_cmds_;
      std::vector<double> stamps;

      std::ofstream sync_csv_file;

      // Synchronized vehicle data callback
      void syncCallback(
        const double stamp,
        const SpeedState speed_state,
        const double cmd_vel
      ) {
        // Store data
        current_states_.push_back(speed_state);
        current_cmds_.push_back(cmd_vel);
        stamps.push_back(stamp);
      }

      void logData(std::vector<SpeedState>& pred) {
        if (!write_to_file_)
          return;
        
        if (!sync_csv_file.is_open()) {
          sync_csv_file.open("speed_sync.csv");
          sync_csv_file << "Absolute Stamp(s), Stamp (s), Cmd Vel (m/s),"
            << " Actual lon-vel (m/s), Actual lon-accel (m/s),"
            << " Actual f-wheel rate (rad/s), Actual r-wheel rate (rad/s),"

            << " Pred lon-vel (m/s),"
            << " Pred f-wheel rate (rad/s), Pred r-wheel rate (rad/s)," 
            
            << " Pred Err lon-vel (m/s),"
            << " Pred Err f-wheel rate (rad/s), Pred Err r-wheel rate (rad/s)" 
            << std::endl;
          sync_csv_file << std::setprecision(20);
        }

        for (size_t n = 0; n < stamps.size(); n++) {
          
          sync_csv_file << stamps[n] << "," << stamps[n] - stamps[0] << "," << current_cmds_[n] << ","
            << current_states_[n].longitudinal_vel << "," << current_states_[n].longitudinal_accel << ", "
            << current_states_[n].front_wheel_angular_vel << "," << current_states_[n].rear_wheel_angular_vel << ","
            
            << pred[n].longitudinal_vel << ","
            << pred[n].front_wheel_angular_vel << "," << pred[n].rear_wheel_angular_vel << ","

            << current_states_[n].longitudinal_vel - pred[n].longitudinal_vel << ","
            << current_states_[n].front_wheel_angular_vel - pred[n].front_wheel_angular_vel << "," << current_states_[n].rear_wheel_angular_vel - pred[n].rear_wheel_angular_vel
            << std::endl;
        }
      }

      std::vector<SpeedState> predictSpeedFull(lib_vehicle_model::VehicleMotionModel& pcm, SpeedState initial_state, double target, bool applyControl, double timestep, double stepCount) const {
        lib_vehicle_model::VehicleState vs;
        vs.longitudinal_vel = initial_state.longitudinal_vel;
        vs.front_wheel_rotation_rate = initial_state.front_wheel_angular_vel;
        vs.rear_wheel_rotation_rate = initial_state.rear_wheel_angular_vel;

        lib_vehicle_model::VehicleControlInput input;
        input.target_velocity = target;
        std::vector<lib_vehicle_model::VehicleControlInput> control_inputs(stepCount, input);
        // NOTE: You must run this through the vehicle model in order to for integration to be done which converts our steer rate to a steer value
        std::vector<lib_vehicle_model::VehicleState> result;
        if (applyControl) {
          result = pcm.predict(vs,control_inputs,timestep);
        } else {
          result = pcm.predict(vs,timestep,timestep * stepCount);
        }
        std::vector<SpeedState> results;
        for (lib_vehicle_model::VehicleState vs : result) {
          SpeedState ss;
          ss.longitudinal_vel = vs.longitudinal_vel;
          ss.front_wheel_angular_vel = vs.front_wheel_rotation_rate;
          ss.rear_wheel_angular_vel = vs.rear_wheel_rotation_rate;
          results.push_back(ss);
        }
        return results;
      }

      std::tuple<double, double> evaluateData(lib_vehicle_model::VehicleMotionModel& pcm) {
        std::vector<SpeedState> forcasts;
        std::vector<double> front_wheel_forcast, front_wheel_current_states;
        forcasts.reserve(current_cmds_.size());
        double timestep = 0.05;
              
        double prevSetpoint = 0; 
        SpeedState initialSpeed;
        bool firstLoop = true;
        int subCount = 0;
        for (size_t n = 0; n < current_cmds_.size(); n++) {
          front_wheel_current_states.push_back(current_states_[n].front_wheel_angular_vel);
          subCount++;
          if (firstLoop) {
            initialSpeed = current_states_[n];
            prevSetpoint = current_cmds_[n];
            firstLoop = false;
          } else if (current_cmds_[n] != prevSetpoint || n == current_cmds_.size() - 1) {

            std::vector<SpeedState> subForcast = predictSpeedFull(pcm, initialSpeed, prevSetpoint, true, timestep, subCount);

            forcasts.insert( forcasts.end(), subForcast.begin(), subForcast.end() ); // Add prediction for this setpoint to forcasts list
            for (SpeedState ss : subForcast) {
              front_wheel_forcast.push_back(ss.front_wheel_angular_vel);
            }
            initialSpeed = current_states_[n];
            subCount = 0;
          }
          prevSetpoint = current_cmds_[n];
        }

        last_forcast_ = forcasts;

        return model_test_tools::getMaxErrorAndRMSE(front_wheel_forcast,front_wheel_current_states);
      }
    };

}