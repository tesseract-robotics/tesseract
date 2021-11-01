#ifndef IIWA7_IKFAST_KINDEMATICS_H
#define IIWA7_IKFAST_KINDEMATICS_H

#include <Eigen/Geometry>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
#include <vector>

namespace tesseract_kinematics
{
class iiwa7Kinematics : public IKFastInvKin
{
public:
  iiwa7Kinematics() = default;  // NOLINT
  iiwa7Kinematics(std::string base_link_name,
                  std::string tip_link_name,
                  std::vector<std::string> joint_names,
                  std::string solver_name = "IKFastInvKin",
                  std::vector<std::vector<double>> free_joint_combos = { { 0.0 } });
};

}  // namespace tesseract_kinematics
#endif  // IIWA7_IKFAST_KINDEMATICS_H
