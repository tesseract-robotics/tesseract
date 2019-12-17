/**
 * @file descartes_tesseract_kinematics.h
 * @brief Implememntatino of a wrapper around tesseract kinematics for the descartes_light kinematics interface
 *
 * @author Matthew Powelson
 * @author Levi Armstrong
 * @date September 17, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_TESSERACT_KINEMATICS_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_TESSERACT_KINEMATICS_HPP

#include <tesseract_motion_planners/descartes/descartes_tesseract_kinematics.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_motion_planners
{
template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                                 std::vector<FloatType>& solution_set) const
{
  return ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
}

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::ik(
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
    const descartes_light::IsValidFn<FloatType>& is_valid_fn,
    const descartes_light::GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
    std::vector<FloatType>& solution_set) const
{
  auto dof = static_cast<int>(tesseract_ik_->numJoints());

  // Convert to appropriate Eigen types
  Eigen::Isometry3d p_double;
  p_double = p.template cast<double>();
  Eigen::VectorXd solution_eigen;

  // Solve IK
  if (!tesseract_ik_->calcInvKin(solution_eigen, p_double, ik_seed_))
    return false;

  // Convert back to a float array
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> solution_float_type;
  solution_float_type = solution_eigen.template cast<FloatType>();
  FloatType* sol = solution_float_type.data();

  // Apply is_valid_fn and redundant_sol_fn
  if (is_valid_fn && redundant_sol_fn)
  {
    if (is_valid_fn_(sol))
      solution_set.insert(end(solution_set), sol, sol + dof);  // If good then add to solution set

    std::vector<FloatType> redundant_sols = redundant_sol_fn(sol);
    if (!redundant_sols.empty())
    {
      int num_sol = static_cast<int>(redundant_sols.size()) / dof;
      for (int s = 0; s < num_sol; ++s)
      {
        FloatType* redundant_sol = redundant_sols.data() + dof * s;
        if (is_valid_fn_(redundant_sol))
          solution_set.insert(end(solution_set), redundant_sol, redundant_sol + dof);  // If good then add to solution
                                                                                       // set
      }
    }
  }
  else if (is_valid_fn && !redundant_sol_fn)
  {
    if (is_valid_fn(sol))
      solution_set.insert(end(solution_set), sol, sol + dof);  // If good then add to solution set
    else
    {
      // If it failed the is_valid_fn get solution that is +/-pi and retry
      descartes_light::harmonizeTowardZero<FloatType>(sol, dof);
      if (is_valid_fn(sol))
        solution_set.insert(end(solution_set), sol, sol + dof);
    }
  }
  else if (!is_valid_fn && redundant_sol_fn)
  {
    solution_set.insert(end(solution_set), sol, sol + dof);  // If good then add to solution set

    std::vector<FloatType> redundant_sols = redundant_sol_fn(sol);
    if (!redundant_sols.empty())
    {
      long num_sol = static_cast<long>(redundant_sols.size()) / dof;
      for (long s = 0; s < num_sol; ++s)
      {
        FloatType* redundant_sol = redundant_sols.data() + dof * s;
        solution_set.insert(end(solution_set), redundant_sol, redundant_sol + dof);  // If good then add to solution
                                                                                     // set
      }
    }
  }
  else
  {
    solution_set.insert(end(solution_set), sol, sol + dof);
  }
  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::fk(const FloatType* pose,
                                                 Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  assert(pose);

  // Convert the Array to an Eigen VectorXd
  Eigen::VectorXd joints(tesseract_fk_->numJoints());
  for (int i = 0; static_cast<unsigned int>(i) < tesseract_fk_->numJoints(); i++)
    joints(i, 0) = pose[i];

  // Get the solution from the Tesseract Kinematics
  Eigen::Isometry3d solution_double;
  bool success = tesseract_fk_->calcFwdKin(solution_double, joints);

  // Cast from double to FloatType
  solution = solution_double.cast<FloatType>();
  return success;
}

template <typename FloatType>
int DescartesTesseractKinematics<FloatType>::dof() const
{
  assert(tesseract_fk_->numJoints() < std::numeric_limits<int>::max());
  return static_cast<int>(tesseract_fk_->numJoints());
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");

  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  std::string valid_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(valid_fn_defined.c_str());
  std::string redundant_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(redundant_fn_defined.c_str());

  std::vector<FloatType> solution_set;
  ik(p, nullptr, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling without functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling with only IsValid functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, nullptr, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with only Redundant Solutions functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with both functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::setIKSeed(
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1> >& seed)
{
  assert(static_cast<int>(seed.size()) == dof());
  ik_seed_ = seed.template cast<double>();
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::setIKSeed(const std::vector<FloatType>& seed)
{
  assert(static_cast<int>(seed.size()) == dof());
  std::vector<double> seed_copy;
  seed_copy.reserve(seed.size());
  for (auto& i : seed)
    seed_copy.push_back(static_cast<double>(i));
  ik_seed_ = Eigen::Map<Eigen::VectorXd>(seed_copy.data(), static_cast<long>(seed_copy.size()));
}
}  // namespace tesseract_motion_planners
#endif
