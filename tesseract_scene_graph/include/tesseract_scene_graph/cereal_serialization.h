#ifndef TESSERACT_SCENE_GRAPH_CEREAL_SERIALIZATION_H
#define TESSERACT_SCENE_GRAPH_CEREAL_SERIALIZATION_H

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_scene_graph/graph.h>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/polymorphic.hpp>

#include <tesseract_common/cereal_serialization.h>
#include <tesseract_geometry/cereal_serialization.h>

namespace tesseract_scene_graph
{
template <class Archive>
void serialize(Archive& ar, SceneState& obj)
{
  ar(cereal::make_nvp("joints", obj.joints));
  ar(cereal::make_nvp("floating_joints", obj.floating_joints));
  ar(cereal::make_nvp("link_transforms", obj.link_transforms));
  ar(cereal::make_nvp("joint_transforms", obj.joint_transforms));
}

template <class Archive>
void serialize(Archive& ar, Material& obj)
{
  ar(cereal::make_nvp("texture_filename", obj.texture_filename));
  ar(cereal::make_nvp("color", obj.color));
  ar(cereal::make_nvp("name", obj.name_));
}

template <class Archive>
void serialize(Archive& ar, Inertial& obj)
{
  ar(cereal::make_nvp("origin", obj.origin));
  ar(cereal::make_nvp("mass", obj.mass));
  ar(cereal::make_nvp("ixx", obj.ixx));
  ar(cereal::make_nvp("ixy", obj.ixy));
  ar(cereal::make_nvp("ixz", obj.ixz));
  ar(cereal::make_nvp("iyy", obj.iyy));
  ar(cereal::make_nvp("iyz", obj.iyz));
  ar(cereal::make_nvp("iyz", obj.izz));
}

template <class Archive>
void serialize(Archive& ar, Visual& obj)
{
  ar(cereal::make_nvp("origin", obj.origin));
  ar(cereal::make_nvp("geometry", obj.geometry));
  ar(cereal::make_nvp("material", obj.material));
  ar(cereal::make_nvp("name", obj.name));
}

template <class Archive>
void serialize(Archive& ar, Collision& obj)
{
  ar(cereal::make_nvp("origin", obj.origin));
  ar(cereal::make_nvp("geometry", obj.geometry));
  ar(cereal::make_nvp("name", obj.name));
}

template <class Archive>
void serialize(Archive& ar, Link& obj)
{
  ar(cereal::make_nvp("inertial", obj.inertial));
  ar(cereal::make_nvp("visual", obj.visual));
  ar(cereal::make_nvp("collision", obj.collision));
  ar(cereal::make_nvp("visible", obj.visible));
  ar(cereal::make_nvp("collision_enabled", obj.collision_enabled));
  ar(cereal::make_nvp("hash", obj.hash_));
  ar(cereal::make_nvp("name", obj.name_));
}

template <class Archive>
void serialize(Archive& ar, JointDynamics& obj)
{
  ar(cereal::make_nvp("damping", obj.damping));
  ar(cereal::make_nvp("friction", obj.friction));
}

template <class Archive>
void serialize(Archive& ar, JointLimits& obj)
{
  ar(cereal::make_nvp("lower", obj.lower));
  ar(cereal::make_nvp("upper", obj.upper));
  ar(cereal::make_nvp("effort", obj.effort));
  ar(cereal::make_nvp("velocity", obj.velocity));
  ar(cereal::make_nvp("acceleration", obj.acceleration));
  ar(cereal::make_nvp("jerk", obj.jerk));
}

template <class Archive>
void serialize(Archive& ar, JointSafety& obj)
{
  ar(cereal::make_nvp("soft_upper_limit", obj.soft_upper_limit));
  ar(cereal::make_nvp("soft_lower_limit", obj.soft_lower_limit));
  ar(cereal::make_nvp("k_position", obj.k_position));
  ar(cereal::make_nvp("k_velocity", obj.k_velocity));
}

template <class Archive>
void serialize(Archive& ar, JointCalibration& obj)
{
  ar(cereal::make_nvp("reference_position", obj.reference_position));
  ar(cereal::make_nvp("rising", obj.rising));
  ar(cereal::make_nvp("falling", obj.falling));
}

template <class Archive>
void serialize(Archive& ar, JointMimic& obj)
{
  ar(cereal::make_nvp("offset", obj.offset));
  ar(cereal::make_nvp("multiplier", obj.multiplier));
  ar(cereal::make_nvp("joint_name", obj.joint_name));
}

template <class Archive>
void serialize(Archive& ar, Joint& obj)
{
  ar(cereal::make_nvp("type", obj.type));
  ar(cereal::make_nvp("axis", obj.axis));
  ar(cereal::make_nvp("child_link_name", obj.child_link_name));
  ar(cereal::make_nvp("parent_link_name", obj.parent_link_name));
  ar(cereal::make_nvp("parent_to_joint_origin_transform", obj.parent_to_joint_origin_transform));
  ar(cereal::make_nvp("dynamics", obj.dynamics));
  ar(cereal::make_nvp("limits", obj.limits));
  ar(cereal::make_nvp("safety", obj.safety));
  ar(cereal::make_nvp("calibration", obj.calibration));
  ar(cereal::make_nvp("mimic", obj.mimic));
  ar(cereal::make_nvp("name", obj.name_));
}

template <class Archive>
void serialize(Archive& ar, SceneGraph& obj)
{
  if (Archive::is_loading::value)
  {
    std::string name;
    ar(cereal::make_nvp("name", name));
    obj.setName(name);

    std::vector<std::shared_ptr<Link>> links;
    ar(cereal::make_nvp("links", links));

    std::vector<std::shared_ptr<Joint>> joints;
    ar(cereal::make_nvp("joints", joints));

    for (const auto& link : links)
      obj.addLinkHelper(link, false);

    for (const auto& joint : joints)
      obj.addJointHelper(joint);

    ar(cereal::make_nvp("acm", obj.acm_));

    std::string root_link_name;
    ar(cereal::make_nvp("root_link_name", root_link_name));
    obj.setRoot(root_link_name);
  }
  else
  {
    std::string name = obj.getName();
    ar(cereal::make_nvp("name", name));

    std::vector<std::shared_ptr<Link>> links;
    links.reserve(obj.link_map_.size());
    for (const auto& pair : obj.link_map_)
      links.push_back(pair.second.first);

    ar(cereal::make_nvp("links", links));

    std::vector<std::shared_ptr<Joint>> joints;
    joints.reserve(obj.joint_map_.size());
    for (const auto& pair : obj.joint_map_)
      joints.push_back(pair.second.first);

    ar(cereal::make_nvp("joints", joints));

    ar(cereal::make_nvp("acm", obj.acm_));

    std::string root_link_name = obj.getRoot();
    ar(cereal::make_nvp("root_link_name", root_link_name));
  }
}
}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_CEREAL_SERIALIZATION_H
