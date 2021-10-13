/*********************************************************************
 *
 * Provides forward and inverse kinematics for Univeral robot designs
 * Author: Kelsey Hawkins (kphawkins@gatech.edu)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <tesseract_kinematics/ur/ur_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
// LCOV_EXCL_START
namespace
{
const double ZERO_THRESH = 0.00000001;
int SIGN(double x) { return (x > 0) - (x < 0); }  // NOLINT
const double PI = M_PI;
}  // namespace

int inverse(const Eigen::Isometry3d& T, const URParameters& params, double* q_sols, double q6_des)
{
  int num_sols = 0;
  double T02 = T(0, 2);
  double T00 = T(0, 0);
  double T01 = T(0, 1);
  double T03 = T(0, 3);
  double T12 = T(1, 2);
  double T10 = T(1, 0);
  double T11 = T(1, 1);
  double T13 = T(1, 3);
  double T22 = T(2, 2);
  double T20 = T(2, 0);
  double T21 = T(2, 1);
  double T23 = T(2, 3);

  ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
  double q1[2];  // NOLINT
  {
    double A = params.d6 * T12 - T13;
    double B = params.d6 * T02 - T03;
    double R = A * A + B * B;
    if (fabs(A) < ZERO_THRESH)
    {
      double div{ 0 };
      if (fabs(fabs(params.d4) - fabs(B)) < ZERO_THRESH)
        div = -SIGN(params.d4) * SIGN(B);
      else
        div = -params.d4 / B;
      double arcsin = asin(div);
      if (fabs(arcsin) < ZERO_THRESH)
        arcsin = 0.0;
      if (arcsin < 0.0)
        q1[0] = arcsin + 2.0 * PI;
      else
        q1[0] = arcsin;
      q1[1] = PI - arcsin;
    }
    else if (fabs(B) < ZERO_THRESH)
    {
      double div{ 0 };
      if (fabs(fabs(params.d4) - fabs(A)) < ZERO_THRESH)
        div = SIGN(params.d4) * SIGN(A);
      else
        div = params.d4 / A;
      double arccos = acos(div);
      q1[0] = arccos;
      q1[1] = 2.0 * PI - arccos;
    }
    else if (params.d4 * params.d4 > R)
    {
      return num_sols;
    }
    else
    {
      double arccos = acos(params.d4 / sqrt(R));
      double arctan = atan2(-B, A);
      double pos = arccos + arctan;
      double neg = -arccos + arctan;
      if (fabs(pos) < ZERO_THRESH)
        pos = 0.0;
      if (fabs(neg) < ZERO_THRESH)
        neg = 0.0;
      if (pos >= 0.0)
        q1[0] = pos;
      else
        q1[0] = 2.0 * PI + pos;
      if (neg >= 0.0)
        q1[1] = neg;
      else
        q1[1] = 2.0 * PI + neg;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
  double q5[2][2];  // NOLINT
  {
    for (int i = 0; i < 2; i++)
    {
      double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - params.d4);
      double div{ 0 };
      if (fabs(fabs(numer) - fabs(params.d6)) < ZERO_THRESH)
        div = SIGN(numer) * SIGN(params.d6);
      else
        div = numer / params.d6;
      double arccos = acos(div);
      q5[i][0] = arccos;
      q5[i][1] = 2.0 * PI - arccos;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////

  {
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        double c1 = cos(q1[i]), s1 = sin(q1[i]);
        double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
        double q6{ 0 };
        ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
        if (fabs(s5) < ZERO_THRESH)
          q6 = q6_des;
        else
        {
          q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1), SIGN(s5) * (T00 * s1 - T10 * c1));
          if (fabs(q6) < ZERO_THRESH)
            q6 = 0.0;
          if (q6 < 0.0)
            q6 += 2.0 * PI;
        }
        ////////////////////////////////////////////////////////////////////////////////

        double q2[2], q3[2], q4[2];  // NOLINT
        ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
        double c6 = cos(q6), s6 = sin(q6);
        double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
        double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
        double p13x = params.d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) -
                      params.d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1;
        double p13y = T23 - params.d1 - params.d6 * T22 + params.d5 * (T21 * c6 + T20 * s6);

        double c3 =
            (p13x * p13x + p13y * p13y - params.a2 * params.a2 - params.a3 * params.a3) / (2.0 * params.a2 * params.a3);
        if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
          c3 = SIGN(c3);
        else if (fabs(c3) > 1.0)
        {
          // TODO NO SOLUTION
          continue;
        }
        double arccos = acos(c3);
        q3[0] = arccos;
        q3[1] = 2.0 * PI - arccos;
        double denom = params.a2 * params.a2 + params.a3 * params.a3 + 2 * params.a2 * params.a3 * c3;
        double s3 = sin(arccos);
        double A = (params.a2 + params.a3 * c3), B = params.a3 * s3;
        q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
        q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
        double c23_0 = cos(q2[0] + q3[0]);
        double s23_0 = sin(q2[0] + q3[0]);
        double c23_1 = cos(q2[1] + q3[1]);
        double s23_1 = sin(q2[1] + q3[1]);
        q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
        q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
        ////////////////////////////////////////////////////////////////////////////////
        for (int k = 0; k < 2; k++)
        {
          if (fabs(q2[k]) < ZERO_THRESH)
            q2[k] = 0.0;
          else if (q2[k] < 0.0)
            q2[k] += 2.0 * PI;
          if (fabs(q4[k]) < ZERO_THRESH)
            q4[k] = 0.0;
          else if (q4[k] < 0.0)
            q4[k] += 2.0 * PI;
          q_sols[num_sols * 6 + 0] = q1[i];
          q_sols[num_sols * 6 + 1] = q2[k];
          q_sols[num_sols * 6 + 2] = q3[k];
          q_sols[num_sols * 6 + 3] = q4[k];
          q_sols[num_sols * 6 + 4] = q5[i][j];
          q_sols[num_sols * 6 + 5] = q6;
          num_sols++;
        }
      }
    }
  }
  return num_sols;
}
// LCOV_EXCL_STOP

URInvKin::URInvKin(URParameters params,
                   std::string base_link_name,
                   std::string tip_link_name,
                   std::vector<std::string> joint_names,
                   std::string solver_name)
  : params_(params)
  , base_link_name_(std::move(base_link_name))
  , tip_link_name_(std::move(tip_link_name))
  , joint_names_(std::move(joint_names))
  , solver_name_(std::move(solver_name))
{
  if (joint_names_.size() != 6)
    throw std::runtime_error("OPWInvKin, only support six joints!");
}

InverseKinematics::UPtr URInvKin::clone() const { return std::make_unique<URInvKin>(*this); }

URInvKin::URInvKin(const URInvKin& other) { *this = other; }

URInvKin& URInvKin::operator=(const URInvKin& other)
{
  base_link_name_ = other.base_link_name_;
  tip_link_name_ = other.tip_link_name_;
  joint_names_ = other.joint_names_;
  params_ = other.params_;
  solver_name_ = other.solver_name_;

  return *this;
}

IKSolutions URInvKin::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                 const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  assert(tip_link_poses.size() == 1);
  assert(tip_link_poses.find(tip_link_name_) != tip_link_poses.end());

  Eigen::Isometry3d base_offset = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d corrected_pose = base_offset.inverse() * tip_link_poses.at(tip_link_name_);

  // Do the analytic IK
  // NOLINTNEXTLINE
  std::array<std::array<double, 6>, 8> sols;  // maximum of 8 IK solutions
  auto num_sols = static_cast<std::size_t>(inverse(corrected_pose, params_, sols[0].data(), 0));

  // Check the output
  IKSolutions solution_set;
  solution_set.reserve(num_sols);
  for (std::size_t i = 0; i < num_sols; ++i)
  {
    Eigen::Map<Eigen::VectorXd> eigen_sol(sols[i].data(), static_cast<Eigen::Index>(sols[i].size()));

    // Harmonize between [-PI, PI]
    harmonizeTowardZero<double>(eigen_sol);  // Modifies 'sol' in place

    // Add solution
    solution_set.push_back(eigen_sol);
  }

  return solution_set;
}

Eigen::Index URInvKin::numJoints() const { return 6; }
std::vector<std::string> URInvKin::getJointNames() const { return joint_names_; }
std::string URInvKin::getBaseLinkName() const { return base_link_name_; }
std::string URInvKin::getWorkingFrame() const { return base_link_name_; }
std::vector<std::string> URInvKin::getTipLinkNames() const { return { tip_link_name_ }; }
std::string URInvKin::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
