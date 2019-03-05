#pragma once
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
#include <ros/ros.h>
#include <memory>
#include "ParameterServer.h"

namespace lib_vehicle_model {
  /**
   * @class ROSParameterServer
   * @brief A concrete implementation of the ParameterServer interface which uses the ROS parameter server
   * 
   * Supported parameter types match those supported by the ROS 1 C++ parameter interface except for XmlRpc::XmlRpcValue
   */
  class ROSParameterServer: public ParameterServer
  {
    private:
      std::shared_ptr<ros::NodeHandle> nh_;
    public:

      /**
       * @brief Constructor
       * 
       * @param nh A reference to a ROS node handle which is used to access the parameter server
       */ 
      ROSParameterServer(std::shared_ptr<ros::NodeHandle> nh);

      /**
       * @brief Destructor as required by interface
       * 
       */ 
      ~ROSParameterServer();

      //
      // Overriden interface functions
      //

      bool getParam(const std::string& param_key, std::string& output) override;

      bool getParam(const std::string& param_key, double& output) override;

      bool getParam(const std::string& param_key, float& output) override;

      bool getParam(const std::string& param_key, int& output) override;

      bool getParam(const std::string& param_key, bool& output) override;

      bool getParam(const std::string& param_key, std::vector<std::string>& output) override;

      bool getParam(const std::string& param_key, std::vector<double>& output) override;

      bool getParam(const std::string& param_key, std::vector<float>& output) override;

      bool getParam(const std::string& param_key, std::vector<int>& output) override;

      bool getParam(const std::string& param_key, std::vector<bool>& output) override;
  };
}