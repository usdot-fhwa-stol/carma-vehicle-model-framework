/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "sigpack.h"
#include <dynamic_reconfigure/server.h>
#include <temp_filter_node/FilterConfig.h>
#include <jsk_recognition_msgs/HistogramWithRange.h>

using namespace sp;

ros::Publisher imu_pub;
ros::Subscriber imu_sub;


void callback(temp_filter_node::FilterConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f", config.double_param);
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  static IIR_filt<double,double,double> my_filter;
  static bool firstRun = true;
  if (firstRun) {
    arma::vec3 _b;
    _b[0] = 0.222955;
    _b[1] = 0.445910;
    _b[2] = 0.222955;

    arma::vec3 _a;
    _a[0] = 1.000000;
    _a[1] = -0.295200;
    _a[2] = 0.187020;

    my_filter.set_coeffs(_b, _a);
    firstRun = false;
  }


  double new_value = my_filter(msg->linear_acceleration.x);
  sensor_msgs::Imu new_msg;
  new_msg.header = msg->header;
  new_msg.linear_acceleration.x = new_value;
  imu_pub.publish(new_msg);
}	


// Main execution
int main(int argc, char** argv){
  // Initialize node
  ros::init(argc, argv, "temp_filter_node");

  ros::NodeHandle nh_;
  // // Load parameters
  // //base_link_frame_ = p_cnh_.param("base_link_frame_id", base_link_frame_);
  imu_pub = nh_.advertise<sensor_msgs::Imu>("imu_filtered", 10);
  imu_sub = nh_.subscribe("imu_raw", 10, &imuCallback);

  ros::spin();
  return 0;
};