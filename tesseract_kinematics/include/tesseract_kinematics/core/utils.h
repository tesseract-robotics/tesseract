/**
 * @file utils.h
 * @brief Kinematics utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_UTILS_H
#define TESSERACT_KINEMATICS_UTILS_H

#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <console_bridge/console.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

namespace tesseract_kinematics
{

/**
 * @brief Change the base coordinate system of the jacobian
 * @param jacobian The current Jacobian which gets modified in place
 * @param change_base The transform from the base frame of the jacobian to the desired frame.
 */
inline static void jacobianChangeBase(Eigen::Ref<Eigen::MatrixXd> jacobian, const Eigen::Isometry3d& change_base)
{
  assert(jacobian.rows() == 6);
  for (int i=0; i < jacobian.cols(); i++)
  {
    jacobian.col(i).head(3) = change_base.linear() * jacobian.col(i).head(3);
    jacobian.col(i).tail(3) = change_base.linear() * jacobian.col(i).tail(3);
  }
}

/**
 * @brief Change the reference point of the jacobian
 * @param jacobian The current Jacobian which gets modified in place
 * @param ref_point Is expressed in the same base frame of the jacobian
 *                  and is a vector from the old point to the new point.
 */
inline static void jacobianChangeRefPoint(Eigen::Ref<Eigen::MatrixXd> jacobian, const Eigen::Ref<const Eigen::Vector3d>& ref_point)
{
  assert(jacobian.rows() == 6);
  for(int i=0; i < jacobian.cols(); i++)
  {
    jacobian(0, i) += jacobian(4, i) * ref_point(2) - jacobian(5, i) * ref_point(1);
    jacobian(1, i) += jacobian(5, i) * ref_point(0) - jacobian(3, i) * ref_point(2);
    jacobian(2, i) += jacobian(3, i) * ref_point(1) - jacobian(4, i) * ref_point(0);
  }
}

/**
 * @brief Solve equation Ax=b for x
 * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
 * @param A Input matrix (represents Jacobian)
 * @param b Input vector (represents desired pose)
 * @param x Output vector (represents joint values)
 * @return True if solver completes properly
 */
inline static bool solvePInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                             const Eigen::Ref<const Eigen::VectorXd>& b,
                             Eigen::Ref<Eigen::VectorXd> x)
{
  const double eps = 0.00001;  // TODO: Turn into class member var
  const double lambda = 0.01;  // TODO: Turn into class member var

  if ((A.rows() == 0) || (A.cols() == 0))
  {
    CONSOLE_BRIDGE_logError("Empty matrices not supported in solvePinv()");
    return false;
  }

  if (A.rows() != b.size())
  {
    CONSOLE_BRIDGE_logError("Matrix size mismatch: A(%d, %d), b(%d)", A.rows(), A.cols(), b.size());
    return false;
  }

  // Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are
  // real)
  // in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd& U = svd.matrixU();
  const Eigen::VectorXd& Sv = svd.singularValues();
  const Eigen::MatrixXd& V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  // damp inverse with lambda so that inverse doesn't oscillate near solution
  long int nSv = Sv.size();
  Eigen::VectorXd inv_Sv(nSv);
  for (long int i = 0; i < nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
      inv_Sv(i) = 1 / Sv(i);
    else
      inv_Sv(i) = Sv(i) / (Sv(i) * Sv(i) + lambda * lambda);
  }
  x = V * inv_Sv.asDiagonal() * U.transpose() * b;
  return true;
}

/**
 * @brief Calculate Damped Pseudoinverse
 * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
 * @param A Input matrix (represents Jacobian)
 * @param P Output matrix (represents pseudoinverse of A)
 * @param eps Singular value threshold
 * @param lambda Damping factor
 * @return True if Pseudoinverse completes properly
 */
inline static bool dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              Eigen::Ref<Eigen::MatrixXd> P,
                              const double eps = 0.011,
                              const double lambda = 0.01)
{
  if ((A.rows() == 0) || (A.cols() == 0))
  {
    CONSOLE_BRIDGE_logError("Empty matrices not supported in dampedPInv()");
    return false;
  }

  // Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are
  // real)
  // in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd& U = svd.matrixU();
  const Eigen::VectorXd& Sv = svd.singularValues();
  const Eigen::MatrixXd& V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  // damp inverse with lambda so that inverse doesn't oscillate near solution
  long int nSv = Sv.size();
  Eigen::VectorXd inv_Sv(nSv);
  for (long int i = 0; i < nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
      inv_Sv(i) = 1 / Sv(i);
    else
    {
      inv_Sv(i) = Sv(i) / (Sv(i) * Sv(i) + lambda * lambda);
    }
  }
  P = V * inv_Sv.asDiagonal() * U.transpose();
  return true;
}
}
#endif // TESSERACT_KINEMATICS_UTILS_H
