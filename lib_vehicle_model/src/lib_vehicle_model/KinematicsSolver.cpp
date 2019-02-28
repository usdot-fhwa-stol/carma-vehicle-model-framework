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
using namespace lib_vehicle_model;

double KinematicsSolver::solve(const KinematicsProperty output_prop, const KinematicsProperty unavailable_prop,
   const double prop1, const double prop2, const double prop3) {

  const std::string bad_missing_prop_message("Unsupported KinematicProperty passed to solve function as missing type: ");
  const std::string bad_output_prop_message("Unsupported KinematicProperty passed to solve function as output type: ");

  switch(output_prop) {

    case INITIAL_VELOCITY:
      switch(unavailable_prop) {
        case FINAL_VELOCITY:
          // Prop Order: a,d,t
          // v_i = (d/t) - (0.5*a*t)
          return (prop2 / prop3) - (0.5 * prop1 * prop3);
        case ACCELERATION:
          // Prop Order: v_f,d,t
          // v_i = (2*d/t) - v_f
          return (2 * prop2 / prop3) - prop1;
        case DISTANCE:
          // Prop Order: v_f,a,t
          // v_i = v_f - a * t
          return prop1 - (prop2 * prop3);
        case TIME:
          // Prop Order: v_f,a,d
          // v_i = sqrt(v_f^2 - 2*a*d)
          return sqrt(prop1*prop1 - (2 * prop2 * prop3));
        default:
          throwTypeException(bad_missing_prop_message, unavailable_prop);
          break;
      }
      break;

    case FINAL_VELOCITY:
      switch(unavailable_prop) {
        case INITIAL_VELOCITY:
          // Prop Order: a,d,t
          // v_f = d/t + 0.5*a*t
          return (prop2/prop3) + (0.5*prop1*prop3);
        case ACCELERATION:
          // Prop Order: v_i,d,t
          // v_f = 2*d/t - v_i
          return (2*prop2/prop3) - prop1;
        case DISTANCE:
          // Prop Order: v_i,a,t
          // v_f = v_i + a*t
          return prop1 + (prop2*prop3);
        case TIME:
          // Prop Order: v_i,a,d
          // v_f = sqrt(v_i^2 + 2*a*d)
          return sqrt(prop1*prop1 + 2*prop2*prop3);
        default:
          throwTypeException(bad_missing_prop_message, unavailable_prop);
          break;
      }
      break;

    case ACCELERATION:
      switch(unavailable_prop) {
        case INITIAL_VELOCITY:
          // Prop Order: v_f,d,t
          // a = (2/t) * (v_f - (d/t))
          return (2/prop3) * (prop1 - (prop2/prop3));
        case FINAL_VELOCITY:
          // Prop Order: v_i,d,t
          // a = (2/t) * (d/t - v_i)
          return (2/prop3) * ((prop2/prop3) - prop1);
        case DISTANCE:
          // Prop Order: v_i,v_f,t
          // a = (v_f - v_i) / t
          return (prop2 - prop1) / prop3;
        case TIME:
          // Prop Order: v_i,v_f,d
          // a = (v_f^2 - v_i^2) / (2*d)
          return (prop2*prop2 - prop1*prop1) / (2*prop3);
        default:
          throwTypeException(bad_missing_prop_message, unavailable_prop);
          break;
      }
      break;

    case DISTANCE:
      switch(unavailable_prop) {
        case INITIAL_VELOCITY:
          // Prop Order: v_f,a,t
          // d = v_f*t - 0.5*a*t^2
          return (prop1*prop3) - (0.5*prop2*prop3*prop3);
        case FINAL_VELOCITY:
          // Prop Order: v_i,a,t
          // d = v_i*t + 0.5*a*t^2
          return (prop1*prop3) + (0.5*prop2*prop3*prop3);
        case ACCELERATION:
          // Prop Order: v_i,v_f,t
          // d = (v_i + v_f)*t / 2
          return (prop1+prop2) * prop3 / 2;
        case TIME:
          // Prop Order: v_i,v_f,a
          // d = (v_f^2 - v_i^2) / 2*a
          return (prop2*prop2 - prop1*prop1) / (2*prop3);
        default:
          throwTypeException(bad_missing_prop_message, unavailable_prop);
          break;
      }
      break;
      
    case TIME:
      switch(unavailable_prop) {
        case INITIAL_VELOCITY:
          // Prop Order: v_f,a,d
          // t = (v_f - sqrt(v_f^2 - 2*a*d)) / a
          return (prop1 - sqrt(prop1*prop1 - 2*prop2*prop3)) / prop2;
        case FINAL_VELOCITY:
          // Prop Order: v_i,a,d
          // t = (sqrt(v_i^2 + 2*a*d) - v_i) / a
          return (sqrt(prop1*prop1 + 2*prop2*prop3) - prop1) / prop2;
        case ACCELERATION:
          // Prop Order: v_i,v_f,d
          // t = (2*d) / (v_i+v_f)
          return (2*prop3) / (prop1+prop2);
        case DISTANCE:
          // Prop Order: v_i,v_f,a
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

void KinematicsSolver::throwTypeException(const std::string& message, const KinematicsProperty& propertyToNote) {
  std::ostringstream stream(message);
  stream << propertyToNote;
  throw std::invalid_argument(stream.str());
}

