/*
 * Copyright (C) 2019 LEIDOS.
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

#include <gtest/gtest.h>
#include "lib_vehicle_model/KinematicsSolver.h"
#include "lib_vehicle_model/KinematicsProperty.h"

using namespace lib_vehicle_model;

/**
 * Tests the solve function of the KinematicsSolver
 */ 
TEST(KinematicsSolver, solve)
{
  // Values 
  const double d = 43.6;
  const double a = 2.5;
  const double t = 2.4;
  const double v_i = 15.16666666;
  const double v_f = 21.16666666;
  const double error_bound = 0.0000001;

  // Solve for initial velocity
  // Find: v_i, Miss: v_f, Prop Order: a,d,t
  double v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::FINAL_VELOCITY, a,d,t);
  ASSERT_NEAR(v_i, v_i_result, error_bound);
  // Find: v_i, Miss: a,  Prop Order: v_f,d,t
  v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::ACCELERATION, v_f,d,t);
  ASSERT_NEAR(v_i, v_i_result, error_bound);
  // Find: v_i, Miss: d, Prop Order: v_f,a,t
  v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::DISTANCE, v_f,a,t);
  ASSERT_NEAR(v_i, v_i_result, error_bound);
  // Find: v_i, Miss: t, Prop Order: v_f,a,d
  v_i_result = KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::TIME, v_f,a,d);
  ASSERT_NEAR(v_i, v_i_result, error_bound);

  // Solve for final velocity
  // Find: v_f, Miss: v_i, Prop Order: a,d,t
  double v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::INITIAL_VELOCITY, a,d,t);
  ASSERT_NEAR(v_f, v_f_result, error_bound);
  // Find: v_f, Miss: a, Prop Order: v_i,d,t
  v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::ACCELERATION, v_i,d,t);
  ASSERT_NEAR(v_f, v_f_result, error_bound);
  // Find: v_f, Miss: d, Prop Order: v_i,a,t
  v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::DISTANCE, v_i,a,t);
  ASSERT_NEAR(v_f, v_f_result, error_bound);
  // Find: v_f, Miss: t, Prop Order: v_i,a,d
  v_f_result = KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::TIME, v_i,a,d);
  ASSERT_NEAR(v_f, v_f_result, error_bound);

  // Solve for acceleration
  // Find: a, Miss: v_i, Prop Order: v_f,d,t
  double a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::INITIAL_VELOCITY, v_f,d,t);
  ASSERT_NEAR(a, a_result, error_bound);
  // Find: a, Miss: v_f, Prop Order: v_i,d,t
  a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::FINAL_VELOCITY, v_i,d,t);
  ASSERT_NEAR(a, a_result, error_bound);
  // Find: a, Miss: d, Prop Order: v_i,v_f,t
  a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::DISTANCE, v_i,v_f,t);
  ASSERT_NEAR(a, a_result, error_bound);
  // Find: a, Miss: t, Prop Order: v_i,v_f,d
  a_result = KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::TIME, v_i,v_f,d);
  ASSERT_NEAR(a, a_result, error_bound);

  // Solve for distance
  // Find: d, Miss: v_i, Prop Order: v_f,a,t
  double d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::INITIAL_VELOCITY, v_f,a,t);
  ASSERT_NEAR(d, d_result, error_bound);
  // Find: d, Miss: v_f, Prop Order: v_i,a,t
  d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::FINAL_VELOCITY, v_i,a,t);
  ASSERT_NEAR(d, d_result, error_bound);
  // Find: d, Miss: a, Prop Order: v_i,v_f,t
  d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::ACCELERATION, v_i,v_f,t);
  ASSERT_NEAR(d, d_result, error_bound);
  // Find: d, Miss: t, Prop Order: v_i,v_f,a
  d_result = KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::TIME, v_i,v_f,a);
  ASSERT_NEAR(d, d_result, error_bound);

  // Solve for time
  // Find: t, Miss: v_i, Prop Order: v_f,a,d
  double t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::INITIAL_VELOCITY, v_f,a,d);
  ASSERT_NEAR(t, t_result, error_bound);
  // Find: t, Miss: v_f, Prop Order: v_i,a,d
  t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::FINAL_VELOCITY, v_i,a,d);
  ASSERT_NEAR(t, t_result, error_bound);
  // Find: t, Miss: a, Prop Order: v_i,v_f,d
  t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::ACCELERATION, v_i,v_f,d);
  ASSERT_NEAR(t, t_result, error_bound);
  // Find: t, Miss: d, Prop Order: v_i,v_f,a
  t_result = KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::DISTANCE, v_i,v_f,a);
  ASSERT_NEAR(t, t_result, error_bound);
}



/**
 * Tests the exceptions thrown by the solve function of the KinematicsSolver
 */ 
TEST(KinematicsSolver, solve_exceptions)
{
  // Values 
  const double d = 43.6;
  const double a = 2.5;
  const double t = 2.4;
  const double v_i = 15.16666666;
  const double v_f = 21.16666666;
  const double error_bound = 0.0000001;

  // Solve for initial velocity
  // Find: v_i, Miss: v_f, Prop Order: a,d,t
  // Negative d
  double bad_val = -4.0;
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::FINAL_VELOCITY, a,bad_val,t), 
    std::domain_error);
  // Negative t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::FINAL_VELOCITY, a,d,bad_val), 
    std::domain_error);

  
  // Find: v_i, Miss: a,  Prop Order: v_f,d,t
  // Negative v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::ACCELERATION, bad_val,d,t), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::ACCELERATION, v_f,bad_val,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::ACCELERATION, v_f,d,bad_val), 
    std::domain_error);

  
  // Find: v_i, Miss: d, Prop Order: v_f,a,t
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::DISTANCE, bad_val,a,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::DISTANCE, v_f,a,bad_val), 
    std::domain_error);

  // Find: v_i, Miss: t, Prop Order: v_f,a,d
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::TIME, bad_val,a,d), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::INITIAL_VELOCITY, KinematicsProperty::TIME, v_f,a,bad_val), 
    std::domain_error);


  // Solve for final velocity
  // Find: v_f, Miss: v_i, Prop Order: a,d,t
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::INITIAL_VELOCITY, a,bad_val,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::INITIAL_VELOCITY, a,d,bad_val), 
    std::domain_error);

  // Find: v_f, Miss: a, Prop Order: v_i,d,t
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::ACCELERATION, bad_val,d,t), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::ACCELERATION, v_i,bad_val,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::ACCELERATION, v_i,d,bad_val), 
    std::domain_error);

  // Find: v_f, Miss: d, Prop Order: v_i,a,t
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::DISTANCE, bad_val,a,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::DISTANCE, v_i,a,bad_val), 
    std::domain_error);

  // Find: v_f, Miss: t, Prop Order: v_i,a,d
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::TIME, bad_val,a,d), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::FINAL_VELOCITY, KinematicsProperty::TIME, v_i,a,bad_val), 
    std::domain_error);

  // Solve for acceleration
  // Find: a, Miss: v_i, Prop Order: v_f,d,t
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::INITIAL_VELOCITY, bad_val,d,t), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::INITIAL_VELOCITY, v_f,bad_val,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::INITIAL_VELOCITY, v_f,d,bad_val), 
    std::domain_error);

  // Find: a, Miss: v_f, Prop Order: v_i,d,t
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::FINAL_VELOCITY, bad_val,d,t), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::FINAL_VELOCITY, v_i,bad_val,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::FINAL_VELOCITY, v_i,d,bad_val), 
    std::domain_error);

  // Find: a, Miss: d, Prop Order: v_i,v_f,t
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::DISTANCE, bad_val,v_f,t), 
    std::domain_error);
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::DISTANCE, v_i,bad_val,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::DISTANCE, v_i,v_f,bad_val), 
    std::domain_error);

  // Find: a, Miss: t, Prop Order: v_i,v_f,d
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::TIME, bad_val,v_f,d), 
    std::domain_error);
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::TIME, v_i,bad_val,d), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::ACCELERATION, KinematicsProperty::TIME, v_i,v_f,bad_val), 
    std::domain_error);


  // Solve for distance
  // Find: d, Miss: v_i, Prop Order: v_f,a,t
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::INITIAL_VELOCITY, bad_val,a,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::INITIAL_VELOCITY, v_i,a,bad_val), 
    std::domain_error);


  // Find: d, Miss: v_f, Prop Order: v_i,a,t
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::FINAL_VELOCITY, bad_val,a,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::FINAL_VELOCITY, v_i,a,bad_val), 
    std::domain_error);

  // Find: d, Miss: a, Prop Order: v_i,v_f,t
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::ACCELERATION, bad_val,v_f,t), 
    std::domain_error);
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::ACCELERATION, v_i,bad_val,t), 
    std::domain_error);
  // Neg t
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::ACCELERATION, v_i,v_f,bad_val), 
    std::domain_error);

  // Find: d, Miss: t, Prop Order: v_i,v_f,a
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::TIME, bad_val,v_f,a), 
    std::domain_error);
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::TIME, v_i,bad_val,a), 
    std::domain_error);
  // Test bad accel vel mistmatch
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::TIME, 5,10,-10), 
    std::domain_error);
  ASSERT_NO_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::TIME, 5,5,0));
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::DISTANCE, KinematicsProperty::TIME, 5,10,0), 
    std::domain_error);

  // Solve for time
  // Find: t, Miss: v_i, Prop Order: v_f,a,d
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::INITIAL_VELOCITY, bad_val,a,d), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::INITIAL_VELOCITY, v_f,a,bad_val), 
    std::domain_error);

  // Find: t, Miss: v_f, Prop Order: v_i,a,d
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::FINAL_VELOCITY, bad_val,a,d), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::FINAL_VELOCITY, v_i,a,bad_val), 
    std::domain_error);

  // Find: t, Miss: a, Prop Order: v_i,v_f,d
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::ACCELERATION, bad_val,v_f,d), 
    std::domain_error);
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::ACCELERATION, v_i,bad_val,d), 
    std::domain_error);
  // Neg d
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::ACCELERATION, v_i,v_f,bad_val), 
    std::domain_error);


  // Find: t, Miss: d, Prop Order: v_i,v_f,a
  // Neg v_i
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::DISTANCE, bad_val,v_f,a), 
    std::domain_error);
  // Neg v_f
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::DISTANCE, v_i,bad_val,a), 
    std::domain_error);
  // Test bad accel vel mistmatch
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::DISTANCE, 5,10,-10), 
    std::domain_error);
  ASSERT_NO_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::DISTANCE, 5,5,0));
  ASSERT_THROW(KinematicsSolver::solve(KinematicsProperty::TIME, KinematicsProperty::DISTANCE, 5,10,0), 
    std::domain_error);
  
}
