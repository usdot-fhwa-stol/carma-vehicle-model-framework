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
        * @brief Helper function to throw an exception with the provided message appended with the provided KinematicProperty field
        * 
        * @throws std::invalid_argument exception with the provided message and data
        * 
        */
      void throwTypeException(const std::string& message, const KinematicsProperty& propertyToNote) {
        std::ostringstream stream(message);
        stream << propertyToNote;
        throw std::invalid_argument(stream.str());
      }

      /**
       * @brief Helper function to check that the provided value is non-negative
       * 
       * This function should be used to check d and t values are positive
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
          || (a < 0 && v_i < v_f)) {
            std::ostringstream stream;
            stream << "Impossible acceleration and velocity combination a = " << prop3 << " v_i = " << prop1 << " v_f = " << prop2;
            throw std::domain_error(stream.str());
        }
      }
    }

    // 
    // Public Namespace
    //

    double solve(const KinematicsProperty& output_prop, const KinematicsProperty& unavailable_prop,
      const double& prop1, const double& prop2, const double& prop3) {

      const std::string bad_missing_prop_message("Unsupported KinematicProperty passed to solve function as missing type: ");
      const std::string bad_output_prop_message("Unsupported KinematicProperty passed to solve function as output type: ");

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
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_i = (2*d/t) - v_f
              return (2 * prop2 / prop3) - prop1;
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_f,a,t
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_i = v_f - a * t
              return prop1 - (prop2 * prop3);
            case KinematicsProperty::TIME:
              // Prop Order: v_f,a,d
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // v_i = sqrt(v_f^2 - 2*a*d)
              return sqrt(prop1*prop1 - (2 * prop2 * prop3));
            default:
              throwTypeException(bad_missing_prop_message, unavailable_prop);
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
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_f = 2*d/t - v_i
              return (2*prop2/prop3) - prop1;
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_i,a,t
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // v_f = v_i + a*t
              return prop1 + (prop2*prop3);
            case KinematicsProperty::TIME:
              // Prop Order: v_i,a,d
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // v_f = sqrt(v_i^2 + 2*a*d)
              return sqrt(prop1*prop1 + 2*prop2*prop3);
            default:
              throwTypeException(bad_missing_prop_message, unavailable_prop);
              break;
          }
          break;

        case KinematicsProperty::ACCELERATION:
          switch(unavailable_prop) {
            case KinematicsProperty::INITIAL_VELOCITY:
              // Prop Order: v_f,d,t
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // a = (2/t) * (v_f - (d/t))
              return (2/prop3) * (prop1 - (prop2/prop3));
            case KinematicsProperty::FINAL_VELOCITY:
              // Prop Order: v_i,d,t
              checkIsPositive(prop2, KinematicsProperty::DISTANCE);
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // a = (2/t) * (d/t - v_i)
              return (2/prop3) * ((prop2/prop3) - prop1);
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_i,v_f,t
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // a = (v_f - v_i) / t
              return (prop2 - prop1) / prop3;
            case KinematicsProperty::TIME:
              // Prop Order: v_i,v_f,d
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // a = (v_f^2 - v_i^2) / (2*d)
              return (prop2*prop2 - prop1*prop1) / (2*prop3);
            default:
              throwTypeException(bad_missing_prop_message, unavailable_prop);
              break;
          }
          break;

        case KinematicsProperty::DISTANCE:
          switch(unavailable_prop) {
            case KinematicsProperty::INITIAL_VELOCITY:
              // Prop Order: v_f,a,t
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // d = v_f*t - 0.5*a*t^2
              return (prop1*prop3) - (0.5*prop2*prop3*prop3);
            case KinematicsProperty::FINAL_VELOCITY:
              // Prop Order: v_i,a,t
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // d = v_i*t + 0.5*a*t^2
              return (prop1*prop3) + (0.5*prop2*prop3*prop3);
            case KinematicsProperty::ACCELERATION:
              // Prop Order: v_i,v_f,t
              checkIsPositive(prop3, KinematicsProperty::TIME);
              // d = (v_i + v_f)*t / 2
              return (prop1+prop2) * prop3 / 2;
            case KinematicsProperty::TIME:
              // Prop Order: v_i,v_f,a
              checkDeltaVAndAccelMatch(prop1, prop2, prop3);
              // d = (v_f^2 - v_i^2) / 2*a
              return (prop2*prop2 - prop1*prop1) / (2*prop3);
            default:
              throwTypeException(bad_missing_prop_message, unavailable_prop);
              break;
          }
          break;
          
        case KinematicsProperty::TIME:
          switch(unavailable_prop) {
            case KinematicsProperty::INITIAL_VELOCITY:
              // Prop Order: v_f,a,d
              // t = (v_f +- sqrt(v_f^2 - 2*a*d)) / a
              // Need to evaluate both positive and negative v_i from square root
              const double v_f_sqr = prop1*prop1;
              const double 2ad = 2*prop2*prop3;
              const double t1 = (prop1 - sqrt(v_f_sqr - 2ad)) / prop2;
              const double t2 = (prop1 + sqrt(v_f_sqr - 2ad)) / prop2;
              // We cant have a negative change in time so return the value above zero if only 1
              if (t1 < 0) {
                return t2;
              } else if (t2 < 0) {
                return t1;
              }
              // v_i = v_f - a*t
              const double at1 = prop2 * t1;
              const double at2 = prop2 * t2;
              const double v_i1 = prop1 - (at1);
              const double v_i2 = prop1 - (at2);
              // d = v_i*t + 0.5*a*t^2
              const double d_1 = v_i1 + 0.5*at1*t1;
              const double d_2 = v_i2 + 0.5*at2*t2;
              // Compute delta with d since there might be floating point error
              const double delta_1 = abs(prop3 - d_1);
              const double delta_2 = abs(prop3 - d_2);

              return (delta_1 < delta_2) ? t1 : t2; // Return the time which gives the closest distance
            case KinematicsProperty::FINAL_VELOCITY:
              // Prop Order: v_i,a,d
              // t = (sqrt(v_i^2 + 2*a*d) -+ v_i) / a
              // Need to evaluate both positive and negative v_f from square root
              const double v_i_sqr = prop1*prop1;
              const double 2ad = 2*prop2*prop3;
              const double t1 = (sqrt(v_i_sqr + 2ad) - prop1) / prop2;
              const double t2 = (sqrt(v_i_sqr + 2ad) + prop1) / prop2;
              // We cant have a negative change in time so return the value above zero if only 1
              if (t1 < 0) {
                return t2;
              } else if (t2 < 0) {
                return t1;
              }

              // v_f = v_i + a*t
              const double at1 = prop2 * t1;
              const double at2 = prop2 * t2;
              const double v_f1 = prop1 + (at1);
              const double v_f2 = prop1 + (at2);
              // d = 0.5 * (v_i + v_f) * t
              const double d_1 = 0.5 * (prop1 + v_f1) * t1;
              const double d_2 = 0.5 * (prop1 + v_f2) * t2;
              // Compute delta with d since there might be floating point error
              const double delta_1 = abs(prop3 - d_1);
              const double delta_2 = abs(prop3 - d_2);

              return (delta_1 < delta_2) ? t1 : t2;
            case KinematicsProperty::ACCELERATION:
              // Prop Order: v_i,v_f,d
              checkIsPositive(prop3, KinematicsProperty::DISTANCE);
              // t = (2*d) / (v_i+v_f)
              return (2*prop3) / (prop1+prop2);
            case KinematicsProperty::DISTANCE:
              // Prop Order: v_i,v_f,a
              checkDeltaVAndAccelMatch(prop1, prop2, prop3);
              // t = (v_f - v_i) / a
              return (prop2 - prop1) / prop3;
            default:
              throwTypeException(bad_missing_prop_message, unavailable_prop);
              break;
          }
          break;

        // This should never be reached unless enum KinematicProperty enum is changed without updating this function
        default:
          throwTypeException(bad_output_prop_message, output_prop);
          break;
      }
    }
  }
}

