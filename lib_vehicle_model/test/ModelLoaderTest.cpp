/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <memory>
#include <gtest/gtest.h>
#include <string>
#include "../src/lib_vehicle_model/ModelLoader.h"

/**
 * This file unit tests the ConstraintChecker checker class
 */ 

using namespace lib_vehicle_model;

/**
 * Tests the load function of the ModelLoader
 */ 
TEST(ModelLoader, load)
{
  // Load valid vehicle model
  std::string path("test_libs/unittest_vehicle_model_shared_lib.so");
  std::unique_ptr<VehicleMotionModel, ModelLoader::destroy_fnc_ptr> loadedModel(nullptr,nullptr);
  ASSERT_NO_THROW(
    loadedModel = ModelLoader::load(path)
  );

  // Use bad file path
  path.assign("test_libs/fake/fake.so");
  ASSERT_THROW(
    loadedModel = ModelLoader::load(path),
    std::invalid_argument
  );

  // No create function
  path.assign("test_libs/unittest_no_create_function_lib.so");
  ASSERT_THROW(
    loadedModel = ModelLoader::load(path),
    std::invalid_argument
  );

  // No destroy function
  path.assign("test_libs/unittest_no_destroy_function_lib.so");
  ASSERT_THROW(
    loadedModel = ModelLoader::load(path),
    std::invalid_argument
  );
}
