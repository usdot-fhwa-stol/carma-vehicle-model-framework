#pragma once
/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <stdexcept>

namespace lib_vehicle_model {
  /**
   * Exception which represents an invalid attempt to use or modify a vehicle model
   */ 
  class ModelAccessException : public std::logic_error {
      public:
        /**
         * @brief Constructor which stores the specified message
         * 
         * @param message The message to store in the exception
         * 
         */
        ModelAccessException(const std::string& message);

        /**
         * @brief Constructor which stores the specified message
         * 
         * @param message The message to store in the exception
         * 
         */
        ModelAccessException(const char* message);
  };
}
