#pragma once

#include <message_filters/subscriber.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/shared_ptr.hpp>
#include <autoware_msgs/VehicleCmd.h>
#include <automotive_platform_msgs/SteeringFeedback.h>
#include "lib_vehicle_model/VehicleControlInput.h"
#include "lib_vehicle_model/VehicleState.h"
#include "lib_vehicle_model/VehicleMotionModel.h"

namespace test_helper {
  /**
   * Class Based on Section 2.1 in ROS Bag cookbook http://wiki.ros.org/rosbag/Cookbook
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
  };

  /**
   * Helper function to compute the max error and RMSE of forcasts vs observed data
   * returns a tuple where the first value is the max error and the second is the rmse
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

  void getRPYFromPose(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw) {
    tf::Quaternion quat (
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    );

    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
  }

  /**
   * Helper class to process steering test data
   */ 
  class SteeringTestHelper {
    public: 
      const bool write_to_file_ = false;

      SteeringTestHelper(bool write_to_file) : write_to_file_(write_to_file) {}

      ~SteeringTestHelper() {
        sync_csv_file.close();
      }

      std::vector<double> setpoints, current_steers, stamps, lastForcast;

      std::ofstream sync_csv_file;

      // Synchronized vehicle data callback
      void steeringSyncCallback (
        const autoware_msgs::VehicleCmdConstPtr cmd_msg,
        const automotive_platform_msgs::SteeringFeedbackConstPtr feedback_msg
      ) {
        const double radPerSteeringRad = 0.05922;
        double stamp = cmd_msg->header.stamp.toSec();
        double setpoint = cmd_msg->ctrl_cmd.steering_angle;
        double current = feedback_msg->steering_wheel_angle * radPerSteeringRad;

        setpoints.push_back(setpoint);
        current_steers.push_back(current);
        stamps.push_back(stamp);
      }

      std::vector<double> predictSteerFull(lib_vehicle_model::VehicleMotionModel& pcm, double initialSteer, double targetSteer, bool applyControl, double timestep, double stepCount) const {
        lib_vehicle_model::VehicleState vs;
        vs.steering_angle = initialSteer;
        lib_vehicle_model::VehicleControlInput input;
        input.target_steering_angle = targetSteer;
        std::vector<lib_vehicle_model::VehicleControlInput> control_inputs(stepCount, input);
        // NOTE: You must run this through the vehicle model in order to for integration to be done which converts our steer rate to a steer value
        std::vector<lib_vehicle_model::VehicleState> result;
        if (applyControl) {
          result = pcm.predict(vs,control_inputs,timestep);
        } else {
          result = pcm.predict(vs,timestep,timestep * stepCount);
        }
        std::vector<double> steer_results;
        for (lib_vehicle_model::VehicleState vs : result) {
          steer_results.push_back(vs.steering_angle);
        }
        return steer_results;
      }

      void logData(std::vector<double>& pred) {
        if (!write_to_file_)
          return;
        
        if (!sync_csv_file.is_open()) {
          sync_csv_file.open("steering_sync.csv");
          sync_csv_file << "Stamp (s), Setpoint (rad), Current (rad), Pred (rad), Pred Err (rad)" << std::endl;
          sync_csv_file << std::setprecision(20);
        }

        for (size_t n = 0; n < setpoints.size(); n++) {
          double pred_error = pred[n] - current_steers[n];
          sync_csv_file << stamps[n] << ", " << setpoints[n] << ", " << current_steers[n] << ", " << pred[n] << ", " << pred_error << std::endl;
        }
      }

      std::tuple<double, double> evaluateSteeringData(lib_vehicle_model::VehicleMotionModel& pcm) {
        std::vector<double> forcasts;
        forcasts.reserve(setpoints.size());
        double timestep = 0.05;
              
        double prevSetpoint = 0; 
        double initialSteer = 0;
        bool firstLoop = true;
        int subCount = 0;
        for (size_t n = 0; n < setpoints.size(); n++) {

          subCount++;
          if (firstLoop) {
            initialSteer = current_steers[n];
            prevSetpoint = setpoints[n];
            firstLoop = false;
          } else if (setpoints[n] != prevSetpoint || n == setpoints.size() - 1) {

            std::vector<double> subForcast = predictSteerFull(pcm, initialSteer, prevSetpoint, true, timestep, subCount);

            forcasts.insert( forcasts.end(), subForcast.begin(), subForcast.end() ); // Add prediction for this setpoint to forcasts list
            initialSteer = current_steers[n];
            subCount = 0;
          }
          prevSetpoint = setpoints[n];
        }

        lastForcast = forcasts;

        return test_helper::getMaxErrorAndRMSE(forcasts,current_steers);
      }
  };

  class ModelTestHelper {
    public: 
      const bool write_to_file_ = false;

      ModelTestHelper(bool write_to_file) : write_to_file_(write_to_file) {}

      ~ModelTestHelper() {
        sync_csv_file.close();
      }

      std::vector<lib_vehicle_model::VehicleState> current_states_, last_forcast_;
      std::vector<lib_vehicle_model::VehicleControlInput> current_cmds_;
      std::vector<double> stamps;

      std::ofstream sync_csv_file;

      // Synchronized vehicle data callback
      void vehicleStateSyncCallback(
        const geometry_msgs::PoseStampedConstPtr pose_msg,
        const sensor_msgs::ImuConstPtr imu_msg,
        const pacmod_msgs::WheelSpeedRptConstPtr wheel_msg,
        const geometry_msgs::TwistStampedConstPtr twist_msg,
        const autoware_msgs::VehicleCmdConstPtr cmd_msg,
        const automotive_platform_msgs::SteeringFeedbackConstPtr steer_msg
      ) {

        const double radPerSteeringRad = 0.05922;
        double stamp = pose_msg->header.stamp.toSec();
        lib_vehicle_model::VehicleState vs;
        vs.X_pos_global = pose_msg->pose.position.x;
        vs.Y_pos_global = pose_msg->pose.position.y;
        double roll, pitch, yaw;
        getRPYFromPose(pose_msg->pose, roll, pitch, yaw);

        vs.orientation = yaw;
        vs.longitudinal_vel = twist_msg->twist.linear.x;
        // START Lateral Velocity computation
        // NOTE: Lateral velocity is not provided by any sensors. 
        // The compromise is to integrate it from the IMU
        // The integration will be reset when linear velocity is 0 or steering angle is 0 for 5 timesteps
        static double lat_vel = 0;
        static double prev_lat_accel = 0;
        static size_t steers_at_zero = 0;

        bool reset_lat_vel = false;
        if (fabs(twist_msg->twist.linear.x < 0.1)) {
          reset_lat_vel = true;
        } else if (fabs(steer_msg->steering_wheel_angle) < 0.08) { // Less then 5 degrees consider as 0
          steers_at_zero++;
        } else if (fabs(steer_msg->steering_wheel_angle) > 0.08) { // Reset steers at 0 count if greater than 5 deg
          steers_at_zero = 0;
        }
        if (steers_at_zero >= 5) {
          reset_lat_vel = true;
        }
        if (reset_lat_vel) {
          lat_vel = 0;
        } else if (stamps.size() > 0) {
          double dt = stamp - stamps[stamps.size() - 1];
          
          lat_vel += prev_lat_accel * dt;
        }

        //std::cout << "X, " << imu_msg->linear_acceleration.y << ", Y, " << -imu_msg->linear_acceleration.x << std::endl;


        prev_lat_accel = -imu_msg->linear_acceleration.x; // IMU is oriented with +Y forward and +X right but we want +Y to the left
        //vs.lateral_vel = lat_vel; TODO at the moment lateral velocity is set to 0
        vs.lateral_vel = 0;
        // END LATERAL VELOCITY CALC

        vs.yaw_rate = imu_msg->angular_velocity.z;
        vs.front_wheel_rotation_rate = (wheel_msg->front_left_wheel_speed + wheel_msg->front_right_wheel_speed) / 2.0; // Front wheel speed average
        vs.rear_wheel_rotation_rate = (wheel_msg->rear_left_wheel_speed + wheel_msg->rear_right_wheel_speed) / 2.0; // Front wheel speed average
        vs.steering_angle = steer_msg->steering_wheel_angle * radPerSteeringRad; // Convert steering wheel angle to wheel angle

        if (current_cmds_.size() > 0) {
          vs.prev_vel_cmd = current_cmds_[current_cmds_.size() - 1].target_velocity;
          vs.prev_steering_cmd = current_cmds_[current_cmds_.size() - 1].target_steering_angle;
        }

        lib_vehicle_model::VehicleControlInput ctrl_input;
        ctrl_input.target_steering_angle = cmd_msg->ctrl_cmd.steering_angle;
        ctrl_input.target_velocity = cmd_msg->ctrl_cmd.linear_velocity;

        current_states_.push_back(vs);
        current_cmds_.push_back(ctrl_input);
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
            
            << " Pred Err X (m), Pred Err Y (m), Pred Err Yaw (m)," 
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
        double timestep = 0.1; // TODO We may want to compute this based on difference between first and second timestamps
        double max_pred_time = 6; // 6 seconds is the expected maximum prediction time
     
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
            // Predict vehicle motion over just the last 6 seconds
            std::vector<lib_vehicle_model::VehicleControlInput> control_inputs_section(current_cmds_.begin() + pred_start_idx, current_cmds_.begin() + n);
            
            auto subForcast = pcm.predict(current_states_[n], control_inputs_section, timestep);

            forcasts.insert( forcasts.end(), subForcast.begin(), subForcast.end() ); // Add prediction for this setpoint to forcasts list
            prev_pred_end_time = time;
            pred_start_idx = n;
          }
        }

        last_forcast_ = forcasts;

        //TODO we probably want to still compute RMSE and error for each pred return test_helper::getMaxErrorAndRMSE(forcasts,current_steers);
      }
    };

}