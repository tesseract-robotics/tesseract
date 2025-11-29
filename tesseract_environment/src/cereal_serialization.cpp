#include <tesseract_environment/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_environment::EnvironmentContactAllowedValidator)
CEREAL_REGISTER_TYPE(tesseract_environment::EnvironmentPtrAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_environment::EnvironmentConstPtrAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_environment::AddContactManagersPluginInfoCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::AddKinematicsInformationCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::AddLinkCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::AddSceneGraphCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::AddTrajectoryLinkCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeCollisionMarginsCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeJointAccelerationLimitsCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeJointOriginCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeJointPositionLimitsCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeJointVelocityLimitsCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeLinkCollisionEnabledCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeLinkOriginCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ChangeLinkVisibilityCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ModifyAllowedCollisionsCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::MoveJointCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::MoveLinkCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::RemoveAllowedCollisionLinkCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::RemoveJointCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::RemoveLinkCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::ReplaceJointCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::SetActiveContinuousContactManagerCommand)
CEREAL_REGISTER_TYPE(tesseract_environment::SetActiveDiscreteContactManagerCommand)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::ContactAllowedValidator,
                                     tesseract_environment::EnvironmentContactAllowedValidator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_environment::EnvironmentPtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_environment::EnvironmentConstPtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::AddContactManagersPluginInfoCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::AddKinematicsInformationCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::AddLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::AddSceneGraphCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::AddTrajectoryLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::ChangeCollisionMarginsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::ChangeJointAccelerationLimitsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::ChangeJointOriginCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::ChangeJointPositionLimitsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::ChangeJointVelocityLimitsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::ChangeLinkCollisionEnabledCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::ChangeLinkOriginCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::ChangeLinkVisibilityCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::ModifyAllowedCollisionsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::MoveJointCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::MoveLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::RemoveAllowedCollisionLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::RemoveJointCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::RemoveLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command, tesseract_environment::ReplaceJointCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::SetActiveContinuousContactManagerCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_environment::Command,
                                     tesseract_environment::SetActiveDiscreteContactManagerCommand)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_environment_cereal)
// LCOV_EXCL_STOP
