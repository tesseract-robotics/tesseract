#ifndef IIWA7_IKFAST_KINDEMATICS_H
#define IIWA7_IKFAST_KINDEMATICS_H

#include <Eigen/Geometry>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
#include <vector>
#include "iiwa7_ikfast_solver.cpp"

namespace tesseract_kinematics
{
class iiwa7Kinematics : public IKFastInvKin
{
public:
  iiwa7Kinematics() = default;  // NOLINT
  iiwa7Kinematics(const std::string base_link_name,
                  const std::string tip_link_name,
                  const std::vector<std::string> joint_names,
                  const std::string solver_name = "IKFastInvKin",
                  const std::vector<std::vector<double>> free_joint_combos = { { 0.0 } });
};

}  // namespace tesseract_kinematics
#endif  // IIWA7_IKFAST_KINDEMATICS_H
