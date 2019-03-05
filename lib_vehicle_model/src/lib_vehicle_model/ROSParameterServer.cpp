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

#include <string>
#include <vector>
#include "lib_vehicle_model/ROSParameterServer.h"

/**
 * Cpp containing the implementation of ROSParameterServer
 */
using namespace lib_vehicle_model;

// Constructor initializes NodeHandle
ROSParameterServer::ROSParameterServer(std::shared_ptr<ros::NodeHandle> nh) {
  nh_ = nh;
}

ROSParameterServer::~ROSParameterServer() {};

// Every getParam call delegates to the NodeHandle
bool ROSParameterServer::getParam(const std::string& param_key, std::string& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, double& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, float& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, int& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, bool& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, std::vector<std::string>& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, std::vector<double>& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, std::vector<float>& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, std::vector<int>& output) {
  return nh_->getParam(param_key, output);
}

bool ROSParameterServer::getParam(const std::string& param_key, std::vector<bool>& output) {
  return nh_->getParam(param_key, output);
}
