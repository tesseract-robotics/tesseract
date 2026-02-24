#include <tesseract/environment/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::environment::EnvironmentContactAllowedValidator)
CEREAL_REGISTER_TYPE(tesseract::environment::EnvironmentPtrAnyPoly)
CEREAL_REGISTER_TYPE(tesseract::environment::EnvironmentConstPtrAnyPoly)
CEREAL_REGISTER_TYPE(tesseract::environment::AddContactManagersPluginInfoCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::AddKinematicsInformationCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::AddLinkCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::AddSceneGraphCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::AddTrajectoryLinkCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeCollisionMarginsCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeJointAccelerationLimitsCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeJointOriginCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeJointPositionLimitsCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeJointVelocityLimitsCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeLinkCollisionEnabledCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeLinkOriginCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ChangeLinkVisibilityCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ModifyAllowedCollisionsCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::MoveJointCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::MoveLinkCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::RemoveAllowedCollisionLinkCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::RemoveJointCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::RemoveLinkCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::ReplaceJointCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::SetActiveContinuousContactManagerCommand)
CEREAL_REGISTER_TYPE(tesseract::environment::SetActiveDiscreteContactManagerCommand)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::ContactAllowedValidator,
                                     tesseract::environment::EnvironmentContactAllowedValidator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface, tesseract::environment::EnvironmentPtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface,
                                     tesseract::environment::EnvironmentConstPtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::AddContactManagersPluginInfoCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::AddKinematicsInformationCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::AddLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::AddSceneGraphCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::AddTrajectoryLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::ChangeCollisionMarginsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::ChangeJointAccelerationLimitsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::ChangeJointOriginCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::ChangeJointPositionLimitsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::ChangeJointVelocityLimitsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::ChangeLinkCollisionEnabledCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::ChangeLinkOriginCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::ChangeLinkVisibilityCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::ModifyAllowedCollisionsCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::MoveJointCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::MoveLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::RemoveAllowedCollisionLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::RemoveJointCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::RemoveLinkCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command, tesseract::environment::ReplaceJointCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::SetActiveContinuousContactManagerCommand)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::environment::Command,
                                     tesseract::environment::SetActiveDiscreteContactManagerCommand)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_environment_cereal)
// LCOV_EXCL_STOP
