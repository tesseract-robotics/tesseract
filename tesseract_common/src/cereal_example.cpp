#include <cereal/archives/json.hpp>
#include <tesseract_common/cereal_serialization.h>
#include <sstream>

int main()
{
  std::stringstream ss;  // any stream can be used

  {
    cereal::JSONOutputArchive oarchive(ss);  // Create an output archive

    tesseract_common::AllowedCollisionMatrix acm;
    acm.addAllowedCollision("test1", "test2", "reason");

    tesseract_common::CalibrationInfo calibration_info;
    calibration_info.joints["test"] = Eigen::Isometry3d::Identity();

    tesseract_common::CollisionMarginData collision_margin_data;

    std::shared_ptr<tesseract_common::ContactAllowedValidator> validator =
        std::make_shared<tesseract_common::ACMContactAllowedValidator>();

    tesseract_common::JointState joint_state;
    tesseract_common::JointTrajectory joint_trajectory;

    tesseract_common::ManipulatorInfo manip_info;

    tesseract_common::KinematicLimits kin_limits;

    std::shared_ptr<tesseract_common::ResourceLocator> locator =
        std::make_shared<tesseract_common::GeneralResourceLocator>();

    tesseract_common::ProfileDictionary profile_dict;

    tesseract_common::AnyPoly poly = true;
    oarchive(acm,
             calibration_info,
             collision_margin_data,
             validator,
             joint_state,
             joint_trajectory,
             manip_info,
             kin_limits,
             locator,
             profile_dict,
             poly);  // Write the data to the archive
  }                  // archive goes out of scope, ensuring all contents are flushed

  std::cout << ss.str() << "\n";
  tesseract_common::AllowedCollisionMatrix acm;
  tesseract_common::CalibrationInfo calibration_info;
  tesseract_common::CollisionMarginData collision_margin_data;
  std::shared_ptr<tesseract_common::ContactAllowedValidator> validator;
  tesseract_common::JointState joint_state;
  tesseract_common::JointTrajectory joint_trajectory;
  tesseract_common::ManipulatorInfo manip_info;
  tesseract_common::KinematicLimits kin_limits;
  std::shared_ptr<tesseract_common::ResourceLocator> locator;
  tesseract_common::ProfileDictionary profile_dict;
  tesseract_common::AnyPoly poly;
  {
    cereal::JSONInputArchive iarchive(ss);  // Create an input archive

    iarchive(acm,
             calibration_info,
             collision_margin_data,
             validator,
             joint_state,
             joint_trajectory,
             manip_info,
             kin_limits,
             locator,
             profile_dict,
             poly);  // Read the data from the archive
  }
}
