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

#include <math.h> 
#include <ros/assert.h>
#include "lib_vehicle_model/KinematicsSolver.h"
/**
 * Cpp containing the implementation of KinematicsSolver
 */

namespace lib_vehicle_model {
  namespace KinematicsSolver {

    //
    //  Private Namespace
    //
    namespace {

      /**
        * @brief Helper function to fail an assertion with the provided message appended with the provided KinematicProperty field
        * 
        * This function should never be called unless this function was not updated when the Enum was updated
        */
      void assertTypeException(const KinematicsProperty& propertyToNote) {
        std::ostringstream stream;
        stream << propertyToNote;
        ROS_ASSERT_MSG(false, "Unsupported KinematicProperty passed to solve function: %s", stream.str().c_str());
      }

      /**
       * @brief Helper function to check that the provided value is non-negative
       * 
       * This function should be used to check v_i,v_f,d,and t values are positive
       * 
       * @throws std::domain_error if the provided value is negative
       */ 
      void checkIsPositive(const double& val, const KinematicsProperty& prop) {
        if (val < 0.0) {
          std::ostringstream stream;
          stream << "Invalid property value: " << prop << " cannot be negative";
          throw std::domain_error(stream.str());
        }
      }

      /**
       * @brief Helper function to check if the change in velocity matches the direction of acceleration
       * 
       * @param v_i Initial velocity
       * @param v_f Final velocity
       * @param a Acceleration
       * 
       * @throws std::domain_error if the provided value is negative
       */ 
      void checkDeltaVAndAccelMatch(const double& v_i, const double& v_f, const double& a) {
        if ((a > 0 && v_i > v_f)
          || (a < 0 && v_i < v_f)
          || a == 0 && v_i != v_f) {
            std::ostringstream stream;
            stream << "Impossible acceleration and velocity combination v_i = " << v_i << " v_f = " << v_f << " a = " << a;
            throw std::domain_error(stream.str());
        }
      }
    }

    // 
    // Public Namespace
    //

    double solve(const KinematicsProperty& output_prop, const KinematicsProperty& unavailable_prop,
      const double& prop1, const double& prop2, const double& prop3) {

      switch(output_prop) {

        case KinematicsProperty::INITIAL_VELOCITY:
          switch(unavailable_prop) {
            case KinematicsProperty::FINAL_VELOCITY:
              // Prop Order: a,d,t
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_i = (d/t) - (0.5*a*t)
              return (prop2 / prop3) - (0.5 * prop1 * prop3);
            case KinematicsProperty::ACCELERATION:
              // Prop Order: v_f,d,t
              checkIsPositive(prop1, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_i = (2*d/t) - v_f
              return (2 * prop2 / prop3) - prop1;
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_f,a,t
              checkIsPositive(prop1, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_i = v_f - a * t
              return prop1 - (prop2 * prop3);
            case KinematicsProperty::TIME:
            {
              // Prop Order: v_f,a,d
              checkIsPositive(prop1, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // v_i = sqrt(v_f^2 - 2*a*d)
              const double vf_sqr = prop1*prop1;
              const double a2 = prop2*2;
              const double inner_term = vf_sqr - a2*prop3;
              // Check for discontinuity 
              if (inner_term < 0) {
                std::ostringstream stream;
                stream << "Discontinuity in calculation with v_f = " << prop1 << " a = " << prop2 << " d = " << prop3;
                throw std::domain_error(stream.str());
              }
              
              return sqrt(inner_term); // Take positive vi as that is a requirement of this function
            }
            default:
              assertTypeException(unavailable_prop);
              break;
          }
          break;

        case KinematicsProperty::FINAL_VELOCITY:
          switch(unavailable_prop) {
            case KinematicsProperty::INITIAL_VELOCITY:
              // Prop Order: a,d,t
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_f = d/t + 0.5*a*t
              return (prop2/prop3) + (0.5*prop1*prop3);
            case KinematicsProperty::ACCELERATION:
              // Prop Order: v_i,d,t
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_f = 2*d/t - v_i
              return (2*prop2/prop3) - prop1;
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_i,a,t
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_f = v_i + a*t
              return prop1 + (prop2*prop3);
            case KinematicsProperty::TIME:
            {
              // Prop Order: v_i,a,d
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // v_f = sqrt(v_i^2 + 2*a*d)
              // Possible results are -v_f or +v_f
              const double vi_sqr = prop1*prop1;
              const double a2 = 2*prop2;
              const double inner_term = vi_sqr + a2*prop3;
              // Check for discontinuity 
              if (inner_term < 0) {
                std::ostringstream stream;
                stream << "Discontinuity in calculation with v_i = " << prop1 << " a = " << prop2 << " d = " << prop3;
                throw std::domain_error(stream.str());
              }

              return sqrt(inner_term); // Take positive v_f as that is a requirement of this function
            }
            default:
              assertTypeException(unavailable_prop);
              break;
          }
          break;

        case KinematicsProperty::ACCELERATION:
          switch(unavailable_prop) {
            case KinematicsProperty::INITIAL_VELOCITY:
              // Prop Order: v_f,d,t
              checkIsPositive(prop1, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // a = (2/t) * (v_f - (d/t))
              return (2/prop3) * (prop1 - (prop2/prop3));
            case KinematicsProperty::FINAL_VELOCITY:
              // Prop Order: v_i,d,t
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // a = (2/t) * (d/t - v_i)
              return (2/prop3) * ((prop2/prop3) - prop1);
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_i,v_f,t
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // a = (v_f - v_i) / t
              return (prop2 - prop1) / prop3;
            case KinematicsProperty::TIME:
              // Prop Order: v_i,v_f,d
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // a = (v_f^2 - v_i^2) / (2*d)
              return (prop2*prop2 - prop1*prop1) / (2*prop3);
            default:
              assertTypeException(unavailable_prop);
              break;
          }
          break;

        case KinematicsProperty::DISTANCE:
          switch(unavailable_prop) {
            case KinematicsProperty::INITIAL_VELOCITY:
              // Prop Order: v_f,a,t
              checkIsPositive(prop1, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // d = v_f*t - 0.5*a*t^2
              return (prop1*prop3) - (0.5*prop2*prop3*prop3);
            case KinematicsProperty::FINAL_VELOCITY:
              // Prop Order: v_i,a,t
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // d = v_i*t + 0.5*a*t^2
              return (prop1*prop3) + (0.5*prop2*prop3*prop3);
            case KinematicsProperty::ACCELERATION:
              // Prop Order: v_i,v_f,t
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // d = (v_i + v_f)*t / 2
              return (prop1+prop2) * prop3 / 2;
            case KinematicsProperty::TIME:
              // Prop Order: v_i,v_f,a
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::FINAL_VELOCITY);
              checkDeltaVAndAccelMatch(prop1, prop2, prop3);
              // d = (v_f^2 - v_i^2) / 2*a
              return (prop2*prop2 - prop1*prop1) / (2*prop3);
            default:
              assertTypeException(unavailable_prop);
              break;
          }
          break;
          
        case KinematicsProperty::TIME:
          switch(unavailable_prop) {
            case KinematicsProperty::INITIAL_VELOCITY:
            {
              // Prop Order: v_f,a,d
              checkIsPositive(prop1, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // t = (v_f - sqrt(v_f^2 - 2*a*d)) / a
              // Need to evaluate both positive and negative v_i from square root
              const double v_f_sqr = prop1*prop1;
              const double ad_2 = 2*prop2*prop3;
              // Inner term
              const double inner_term = v_f_sqr - ad_2;
              // Check for discontinuity 
              if (inner_term < 0) {
                std::ostringstream stream;
                stream << "Discontinuity in calculation with v_f = " << prop1 << " a = " << prop2 << " d = " << prop3;
                throw std::domain_error(stream.str());
              }
              const double pos_vi = sqrt(inner_term); // Take positive vi as that is a requirement of this function
              return (prop1 - pos_vi) / prop2;
            }
            case KinematicsProperty::FINAL_VELOCITY:
            {
              // Prop Order: v_i,a,d
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // t = (sqrt(v_i^2 + 2*a*d) - v_i) / a
              
              const double v_i_sqr = prop1*prop1;

              // Inner term
              const double inner_term = v_i_sqr + 2*prop2*prop3;
              // Check for discontinuity 
              if (inner_term < 0) {
                std::ostringstream stream;
                stream << "Discontinuity in calculation with v_f = " << prop1 << " a = " << prop2 << " d = " << prop3;
                throw std::domain_error(stream.str());
              }

              const double v_f_pos = sqrt(inner_term); // Take v_f as positive since that is a requirement of this function
              return (v_f_pos - prop1) / prop2;
            }
            case KinematicsProperty::ACCELERATION:
              // Prop Order: v_i,v_f,d
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::FINAL_VELOCITY);
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // t = (2*d) / (v_i+v_f)
              return (2*prop3) / (prop1+prop2);
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_i,v_f,a
              checkIsPositive(prop1, KinematicsProperty::INITIAL_VELOCITY);
              checkIsPositive(prop2, KinematicsProperty::FINAL_VELOCITY);
              checkDeltaVAndAccelMatch(prop1, prop2, prop3);
              // t = (v_f - v_i) / a
              return (prop2 - prop1) / prop3;
            default:
              assertTypeException(unavailable_prop);
              break;
          }
          break;

        // This should never be reached unless enum KinematicProperty enum is changed without updating this function
        default:
          assertTypeException(output_prop);
          break;
      }
    }
  }
}
