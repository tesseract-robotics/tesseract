#include <tesseract/kinematics/factory_utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

namespace tesseract::kinematics
{
namespace
{
// Build "<prefix><sep><tag><tail>" with += accumulation; avoids the temporary-string
// allocations that operator+ chains incur (per performance-inefficient-string-concatenation).
std::string concat4(const std::string& prefix, const char* sep, const std::string& tag, const char* tail)
{
  std::string msg;
  msg.reserve(prefix.size() + tag.size() + 32);
  msg += prefix;
  msg += sep;
  msg += tag;
  msg += tail;
  return msg;
}
}  // namespace

tesseract::common::PluginInfo parsePluginInfo(const YAML::Node& sub_node,
                                              const std::string& error_prefix,
                                              const std::string& block_label)
{
  tesseract::common::PluginInfo info;
  if (YAML::Node n = sub_node["class"])
    info.class_name = n.as<std::string>();
  else
    throw std::runtime_error(concat4(error_prefix, ", '", block_label, "' missing 'class' entry!"));

  if (YAML::Node n = sub_node["config"])
    info.config = n;

  return info;
}

std::map<std::string, std::array<double, 3>>
parseSampleResolutionMap(const YAML::Node& sample_res_node,
                         const tesseract::scene_graph::SceneGraph& scene_graph,
                         const std::string& error_prefix,
                         const std::string& block_label)
{
  std::map<std::string, std::array<double, 3>> sample_res_map;

  for (auto it = sample_res_node.begin(); it != sample_res_node.end(); ++it)
  {
    const YAML::Node& joint = *it;
    std::array<double, 3> values{ 0, 0, 0 };

    std::string joint_name;
    if (YAML::Node n = joint["name"])
      joint_name = n.as<std::string>();
    else
      throw std::runtime_error(concat4(error_prefix, ", '", block_label, "' missing 'name' entry!"));

    if (YAML::Node n = joint["value"])
      values[0] = n.as<double>();
    else
      throw std::runtime_error(concat4(error_prefix, ", '", block_label, "' missing 'value' entry!"));

    auto jnt = scene_graph.getJoint(joint_name);
    if (jnt == nullptr)
      throw std::runtime_error(concat4(error_prefix, ", '", block_label, "' failed to find joint in scene graph!"));
    if (jnt->limits == nullptr)
      throw std::runtime_error(concat4(error_prefix, ", joint '", joint_name, "' has no limits!"));

    values[1] = jnt->limits->lower;
    values[2] = jnt->limits->upper;

    if (YAML::Node min = joint["min"])
      values[1] = min.as<double>();
    if (YAML::Node max = joint["max"])
      values[2] = max.as<double>();

    if (values[1] < jnt->limits->lower)
      throw std::runtime_error(error_prefix + ", sample range minimum is less than joint minimum!");
    if (values[2] > jnt->limits->upper)
      throw std::runtime_error(error_prefix + ", sample range maximum is greater than joint maximum!");
    if (values[1] > values[2])
      throw std::runtime_error(error_prefix + ", sample range is not valid!");

    sample_res_map[joint_name] = values;
  }

  return sample_res_map;
}

}  // namespace tesseract::kinematics
