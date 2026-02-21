#ifndef TESSERACT_ENVIRONMENT_CEREAL_SERIALIZATION_H
#define TESSERACT_ENVIRONMENT_CEREAL_SERIALIZATION_H

#include <tesseract_environment/commands.h>
#include <tesseract_environment/environment.h>

#include <tesseract_common/cereal_serialization.h>
#include <tesseract_geometry/cereal_serialization.h>
#include <tesseract_scene_graph/cereal_serialization.h>
#include <tesseract_srdf/cereal_serialization.h>
#include <tesseract_collision/core/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/chrono.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::environment
{
template <class Archive>
void serialize(Archive& ar, Command& obj)
{
  ar(cereal::make_nvp("type", obj.type_));
}

template <class Archive>
void serialize(Archive& ar, AddContactManagersPluginInfoCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("contact_managers_plugin_info", obj.contact_managers_plugin_info_));
}

template <class Archive>
void serialize(Archive& ar, AddKinematicsInformationCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("kinematics_information", obj.kinematics_information_));
}

template <class Archive>
void serialize(Archive& ar, AddLinkCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("link", obj.link_));
  ar(cereal::make_nvp("joint", obj.joint_));
  ar(cereal::make_nvp("replace_allowed", obj.replace_allowed_));
}

template <class Archive>
void serialize(Archive& ar, AddSceneGraphCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("scene_graph", obj.scene_graph_));
  ar(cereal::make_nvp("joint", obj.joint_));
  ar(cereal::make_nvp("prefix", obj.prefix_));
}

template <class Archive>
void serialize(Archive& ar, AddTrajectoryLinkCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("link_name", obj.link_name_));
  ar(cereal::make_nvp("parent_link_name", obj.parent_link_name_));
  ar(cereal::make_nvp("trajectory", obj.trajectory_));
  ar(cereal::make_nvp("replace_allowed", obj.replace_allowed_));
  ar(cereal::make_nvp("method", obj.method_));
}

template <class Archive>
void serialize(Archive& ar, ChangeCollisionMarginsCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("default_margin", obj.default_margin_));
  ar(cereal::make_nvp("pair_margins", obj.pair_margins_));
  ar(cereal::make_nvp("pair_override_type", obj.pair_override_type_));
}

template <class Archive>
void serialize(Archive& ar, ChangeJointAccelerationLimitsCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("limits", obj.limits_));
}

template <class Archive>
void serialize(Archive& ar, ChangeJointOriginCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("joint_name", obj.joint_name_));
  ar(cereal::make_nvp("origin", obj.origin_));
}

template <class Archive>
void serialize(Archive& ar, ChangeJointPositionLimitsCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("limits", obj.limits_));
}

template <class Archive>
void serialize(Archive& ar, ChangeJointVelocityLimitsCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("limits", obj.limits_));
}

template <class Archive>
void serialize(Archive& ar, ChangeLinkCollisionEnabledCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("link_name", obj.link_name_));
  ar(cereal::make_nvp("enabled", obj.enabled_));
}

template <class Archive>
void serialize(Archive& ar, ChangeLinkOriginCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("link_name", obj.link_name_));
  ar(cereal::make_nvp("origin", obj.origin_));
}

template <class Archive>
void serialize(Archive& ar, ChangeLinkVisibilityCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("link_name", obj.link_name_));
  ar(cereal::make_nvp("enabled", obj.enabled_));
}

template <class Archive>
void serialize(Archive& ar, ModifyAllowedCollisionsCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("type", obj.type_));
  ar(cereal::make_nvp("acm", obj.acm_));
}

template <class Archive>
void serialize(Archive& ar, MoveJointCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("joint_name", obj.joint_name_));
  ar(cereal::make_nvp("parent_link", obj.parent_link_));
}

template <class Archive>
void serialize(Archive& ar, MoveLinkCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("joint", obj.joint_));
}

template <class Archive>
void serialize(Archive& ar, RemoveAllowedCollisionLinkCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("link_name", obj.link_name_));
}

template <class Archive>
void serialize(Archive& ar, RemoveJointCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("joint_name", obj.joint_name_));
}

template <class Archive>
void serialize(Archive& ar, RemoveLinkCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("link_name", obj.link_name_));
}

template <class Archive>
void serialize(Archive& ar, ReplaceJointCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("joint", obj.joint_));
}

template <class Archive>
void serialize(Archive& ar, SetActiveContinuousContactManagerCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("active_contact_manager", obj.active_contact_manager_));
}

template <class Archive>
void serialize(Archive& ar, SetActiveDiscreteContactManagerCommand& obj)
{
  ar(cereal::base_class<Command>(&obj));
  ar(cereal::make_nvp("active_contact_manager", obj.active_contact_manager_));
}

template <class Archive>
void serialize(Archive& ar, EnvironmentContactAllowedValidator& obj)
{
  ar(cereal::make_nvp("scene_graph", obj.scene_graph_));
}

template <class Archive>
void serialize(Archive& ar, Environment& obj)
{
  if (Archive::is_loading::value)
  {
    std::shared_ptr<const tesseract::common::ResourceLocator> resource_locator;
    ar(cereal::make_nvp("resource_locator", resource_locator));

    tesseract::environment::Commands commands;
    ar(cereal::make_nvp("commands", commands));

    int init_revision{ 0 };
    ar(cereal::make_nvp("init_revision", init_revision));

    tesseract::scene_graph::SceneState current_state;
    ar(cereal::make_nvp("current_state", current_state));

    std::chrono::system_clock::time_point timestamp;
    ar(cereal::make_nvp("timestamp", timestamp));

    std::chrono::system_clock::time_point current_state_timestamp;
    ar(cereal::make_nvp("current_state_timestamp", current_state_timestamp));

    // No need to serialize the contact allowed validator because it cannot be modified and is constructed internally
    // from the scene graph

    obj.init(commands, init_revision, timestamp, current_state, current_state_timestamp, resource_locator);
  }
  else
  {
    ar(cereal::make_nvp("resource_locator", obj.getResourceLocator()));
    ar(cereal::make_nvp("commands", obj.getCommandHistory()));
    ar(cereal::make_nvp("init_revision", obj.getInitRevision()));
    ar(cereal::make_nvp("current_state", obj.getState()));
    ar(cereal::make_nvp("timestamp", obj.getTimestamp()));
    ar(cereal::make_nvp("current_state_timestamp", obj.getCurrentStateTimestamp()));
    // No need to serialize the contact allowed validator because it cannot be modified and is constructed internally
    // from the scene graph
  }
}

}  // namespace tesseract::environment

#endif  // TESSERACT_ENVIRONMENT_CEREAL_SERIALIZATION_H
