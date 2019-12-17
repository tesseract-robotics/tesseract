/**
 * @file descartes_tesseract_kinematics.h
 * @brief A wrapper around tesseract kinematics for the descartes_light kinematics interface
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_TESSERACT_KINEMATICS_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_TESSERACT_KINEMATICS_H

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <vector>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

namespace tesseract_motion_planners
{
/** @brief Provides a Descartes interface for Tesseract Kinematics
 */
template <typename FloatType>
class DescartesTesseractKinematics : public descartes_light::KinematicsInterface<FloatType>
{
public:
  using Ptr = std::shared_ptr<DescartesTesseractKinematics>;
  using ConstPtr = std::shared_ptr<const DescartesTesseractKinematics>;

  /**
   * @brief This constructor defaults to using isWithinLimits as the isValidFn and gets redundant solutions within joint
   * limits
   * @param tesseract_fk Forward kinematics object
   * @param tesseract_ik Inverse kinematics object
   */
  DescartesTesseractKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr tesseract_fk,
                               const tesseract_kinematics::InverseKinematics::ConstPtr tesseract_ik)
    : DescartesTesseractKinematics(tesseract_fk,
                                   tesseract_ik,
                                   std::bind(&descartes_light::isWithinLimits<FloatType>,
                                             std::placeholders::_1,
                                             tesseract_fk->getLimits().cast<FloatType>()),
                                   std::bind(&descartes_light::getRedundantSolutions<FloatType>,
                                             std::placeholders::_1,
                                             tesseract_fk->getLimits().cast<FloatType>()))
  {
    ik_seed_ = Eigen::VectorXd::Zero(dof());
  }

  /**
   * @brief DescartesTesseractKinematics
   * @param tesseract_fk Forward kinematics object
   * @param tesseract_ik Inverse kinematic object
   * @param is_valid_fn Function that is used to determine if a vertex is valid
   * @param redundant_sol_fn Function called to get redundant solutions beyond what tesseract_ik returns
   */
  DescartesTesseractKinematics(tesseract_kinematics::ForwardKinematics::ConstPtr tesseract_fk,
                               tesseract_kinematics::InverseKinematics::ConstPtr tesseract_ik,
                               descartes_light::IsValidFn<FloatType> is_valid_fn,
                               descartes_light::GetRedundantSolutionsFn<FloatType> redundant_sol_fn)
    : tesseract_fk_(std::move(tesseract_fk))
    , tesseract_ik_(std::move(tesseract_ik))
    , is_valid_fn_(std::move(is_valid_fn))
    , redundant_sol_fn_(std::move(redundant_sol_fn))
  {
    ik_seed_ = Eigen::VectorXd::Zero(dof());
  }

  /** @brief Calculate inverse kinematics using isValidFn, GetRedundantSolutionsFn, and ik_seed provided

  Note: This currenlty only supports single solutions*/
  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;

  /** @brief Calculate forward kinematics */
  bool fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  /** @brief Returns the number of joints in the kinematics object */
  int dof() const override;

  /** @brief Analyzes the effect of the isValidFn and redundant solutions on the number of returned solutions and prints
   * the results*/
  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

  /** @brief Sets the seed used by inverse kinematics. Must be length dof(). Default: Eigen::VectorXd::Zero(dof())*/
  void setIKSeed(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1> >& seed);

  /** @brief Sets the seed used by inverse kinematics. Must be length dof(). Default: Eigen::VectorXd::Zero(dof())**/
  void setIKSeed(const std::vector<FloatType>& seed);

protected:
  tesseract_kinematics::ForwardKinematics::ConstPtr tesseract_fk_;
  tesseract_kinematics::InverseKinematics::ConstPtr tesseract_ik_;
  descartes_light::IsValidFn<FloatType> is_valid_fn_;
  descartes_light::GetRedundantSolutionsFn<FloatType> redundant_sol_fn_;
  Eigen::VectorXd ik_seed_;

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          const descartes_light::IsValidFn<FloatType>& is_valid_fn,
          const descartes_light::GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
          std::vector<FloatType>& solution_set) const;
};

using DescartesTesseractKinematicsD = DescartesTesseractKinematics<double>;
using DescartesTesseractKinematicsF = DescartesTesseractKinematics<float>;
}  // namespace tesseract_motion_planners

#endif
