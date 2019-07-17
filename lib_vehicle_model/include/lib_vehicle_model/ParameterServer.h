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

namespace lib_vehicle_model {
  /**
   * @class ParameterServer
   * @brief An interface which defines the functions needed to access parameters for use in vehicle models.
   * 
   * Supported parameter types match those supported by the ROS 1 C++ parameter interface except for XmlRpc::XmlRpcValue
   */
  class ParameterServer 
  {
    public:

      /**
       * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
       * 
       */
      virtual ~ParameterServer() {};

      /**
       * @brief Get a string value from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, std::string& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a double value from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, double& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a float value from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, float& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a integer value from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, int& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a boolean value from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, bool& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a list of string values from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, std::vector<std::string>& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a list of double values from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, std::vector<double>& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a list of float values from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, std::vector<float>& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a list of integer values from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, std::vector<int>& output) = 0; // Defined as pure virtual function

      /**
       * @brief Get a list of boolean values from the parameter server
       * 
       * @param param_key The parameter key
       * @param output A reference to the variable to populate with the parameter value
       * 
       * @return True if the parameter exists and was read. False otherwise
       * 
       */
      virtual bool getParam(const std::string& param_key, std::vector<bool>& output) = 0; // Defined as pure virtual function
  };
}