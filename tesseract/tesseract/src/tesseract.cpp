/**
 * @file common.h
 * @brief This is a collection of common methods
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_environment/kdl/kdl_state_solver.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>

namespace tesseract
{
Tesseract::Tesseract() { clear(); }

bool Tesseract::isInitialized() const { return initialized_; }

bool Tesseract::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph)
{
  if (!scene_graph)
    return false;
  clear();
  init_info_ = std::make_shared<TesseractInitInfo>();
  init_info_->type = TesseractInitType::SCENE_GRAPH;
  init_info_->scene_graph = scene_graph->clone();

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::Environment>();
  if (!environment_->init<tesseract_environment::OFKTStateSolver>(*scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }

  auto srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  srdf->getName() = scene_graph->getName();

  manipulator_manager_ = std::make_shared<ManipulatorManager>();
  if (!manipulator_manager_->init(environment_, srdf))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize manipulator manager.");
    return false;
  }

  registerDefaultContactManagers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph,
                     tesseract_scene_graph::SRDFModel::Ptr srdf_model)
{
  if (!scene_graph || !srdf_model)
    return false;
  clear();
  init_info_ = std::make_shared<TesseractInitInfo>();
  init_info_->type = TesseractInitType::SCENE_GRAPH_SRDF_MODEL;
  init_info_->scene_graph = scene_graph->clone();
  init_info_->srdf_model = std::make_shared<tesseract_scene_graph::SRDFModel>(*srdf_model);

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::Environment>();
  if (!environment_->init<tesseract_environment::OFKTStateSolver>(*scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }

  manipulator_manager_ = std::make_shared<ManipulatorManager>();
  if (!manipulator_manager_->init(environment_, std::make_shared<tesseract_scene_graph::SRDFModel>(*srdf_model)))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize manipulator manager.");
    return false;
  }

  registerDefaultContactManagers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const std::string& urdf_string, const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
  init_info_ = std::make_shared<TesseractInitInfo>();
  init_info_->type = TesseractInitType::URDF_STRING;
  init_info_->urdf_string = urdf_string;
  init_info_->resource_locator = locator;

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_urdf::parseURDFString(urdf_string, locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::Environment>();
  if (!environment_->init<tesseract_environment::OFKTStateSolver>(*scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }

  auto srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  srdf->getName() = scene_graph->getName();

  manipulator_manager_ = std::make_shared<ManipulatorManager>();
  if (!manipulator_manager_->init(environment_, srdf))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize manipulator manager.");
    return false;
  }

  registerDefaultContactManagers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const std::string& urdf_string,
                     const std::string& srdf_string,
                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
  init_info_ = std::make_shared<TesseractInitInfo>();
  init_info_->type = TesseractInitType::URDF_STRING_SRDF_STRING;
  init_info_->urdf_string = urdf_string;
  init_info_->srdf_string = srdf_string;
  init_info_->resource_locator = locator;

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_urdf::parseURDFString(urdf_string, locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Parse srdf string into SRDF Model
  tesseract_scene_graph::SRDFModel::Ptr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  if (!srdf->initString(*scene_graph, srdf_string))
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    return false;
  }

  // Add allowed collision matrix to scene graph
  tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

  // Construct Environment
  environment_ = std::make_shared<tesseract_environment::Environment>();
  if (!environment_->init<tesseract_environment::OFKTStateSolver>(*scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }

  manipulator_manager_ = std::make_shared<ManipulatorManager>();
  if (!manipulator_manager_->init(environment_, srdf))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize manipulator manager.");
    return false;
  }

  registerDefaultContactManagers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const boost::filesystem::path& urdf_path,
                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
  init_info_ = std::make_shared<TesseractInitInfo>();
  init_info_->type = TesseractInitType::URDF_PATH;
  init_info_->urdf_path = urdf_path;
  init_info_->resource_locator = locator;

  // Parse urdf file into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_urdf::parseURDFFile(urdf_path.string(), locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::Environment>();
  if (!environment_->init<tesseract_environment::OFKTStateSolver>(*scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }

  auto srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  srdf->getName() = scene_graph->getName();

  manipulator_manager_ = std::make_shared<ManipulatorManager>();
  if (!manipulator_manager_->init(environment_, srdf))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize manipulator manager.");
    return false;
  }

  registerDefaultContactManagers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const boost::filesystem::path& urdf_path,
                     const boost::filesystem::path& srdf_path,
                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
  init_info_ = std::make_shared<TesseractInitInfo>();
  init_info_->type = TesseractInitType::URDF_PATH_SRDF_PATH;
  init_info_->urdf_path = urdf_path;
  init_info_->srdf_path = srdf_path;
  init_info_->resource_locator = locator;

  // Parse urdf file into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_urdf::parseURDFFile(urdf_path.string(), locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Parse srdf file into SRDF Model
  auto srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  if (!srdf->initFile(*scene_graph, srdf_path.string()))
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    return false;
  }

  // Add allowed collision matrix to scene graph
  tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

  // Construct Environment
  environment_ = std::make_shared<tesseract_environment::Environment>();
  if (!environment_->init<tesseract_environment::OFKTStateSolver>(*scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }

  manipulator_manager_ = std::make_shared<ManipulatorManager>();
  if (!manipulator_manager_->init(environment_, srdf))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize manipulator manager.");
    return false;
  }

  registerDefaultContactManagers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const tesseract_environment::Environment& env, const ManipulatorManager& manipulator_manager)
{
  clear();
  init_info_ = std::make_shared<TesseractInitInfo>();
  init_info_->type = TesseractInitType::ENVIRONMENT_MANIPULATOR_MANAGER;
  init_info_->environment = env.clone();
  init_info_->manipulator_manager = manipulator_manager.clone(init_info_->environment);
  environment_ = env.clone();
  manipulator_manager_ = manipulator_manager.clone(environment_);
  initialized_ = true;
  return initialized_;
}

bool Tesseract::init(const TesseractInitInfo::Ptr& init_info)
{
  switch (init_info->type)
  {
    case TesseractInitType::SCENE_GRAPH:
      init(init_info->scene_graph);
      break;
    case TesseractInitType::SCENE_GRAPH_SRDF_MODEL:
      init(init_info->scene_graph, std::make_shared<tesseract_scene_graph::SRDFModel>(*(init_info->srdf_model)));
      break;
    case TesseractInitType::URDF_STRING:
      init(init_info->urdf_string, init_info->resource_locator);
      break;
    case TesseractInitType::URDF_STRING_SRDF_STRING:
      init(init_info->urdf_string, init_info->srdf_string, init_info->resource_locator);
      break;
    case TesseractInitType::URDF_PATH:
      init(init_info->urdf_path, init_info->resource_locator);
      break;
    case TesseractInitType::URDF_PATH_SRDF_PATH:
      init(init_info->urdf_path, init_info->srdf_path, init_info->resource_locator);
      break;
    case TesseractInitType::ENVIRONMENT_MANIPULATOR_MANAGER:
      init(*init_info->environment, *init_info->manipulator_manager);
      break;
    default:
      CONSOLE_BRIDGE_logError("Unsupported TesseractInitInfo type.");
      return false;
  }
  return true;
}

Tesseract::Ptr Tesseract::clone() const
{
  auto clone = std::make_shared<Tesseract>();

  if (environment_)
    clone->environment_ = environment_->clone();

  if (clone->environment_)
    if (manipulator_manager_)
      clone->manipulator_manager_ = manipulator_manager_->clone(clone->environment_);

  clone->init_info_ = init_info_;
  clone->initialized_ = initialized_;
  clone->find_tcp_cb_ = find_tcp_cb_;

  return clone;
}

bool Tesseract::reset() { return init(init_info_); }

void Tesseract::setResourceLocator(tesseract_scene_graph::ResourceLocator::Ptr locator)
{
  init_info_->resource_locator = locator;
}

const tesseract_scene_graph::ResourceLocator::Ptr& Tesseract::getResourceLocator() const
{
  return init_info_->resource_locator;
}

tesseract_environment::Environment::Ptr Tesseract::getEnvironment() { return environment_; }
tesseract_environment::Environment::ConstPtr Tesseract::getEnvironment() const { return environment_; }

ManipulatorManager::Ptr Tesseract::getManipulatorManager() { return manipulator_manager_; }
ManipulatorManager::ConstPtr Tesseract::getManipulatorManager() const { return manipulator_manager_; }

/** @brief registerDefaultContactManagers */
bool Tesseract::registerDefaultContactManagers()
{
  using namespace tesseract_collision;
  if (!environment_)
    return false;

  // Register contact manager
  environment_->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                               &tesseract_collision_bullet::BulletDiscreteBVHManager::create);
  environment_->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                 &tesseract_collision_bullet::BulletCastBVHManager::create);

  // Set Active contact manager
  environment_->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name());
  environment_->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name());

  return true;
}

void Tesseract::clear()
{
  initialized_ = false;
  environment_ = nullptr;
  manipulator_manager_ = nullptr;
  init_info_ = nullptr;
  find_tcp_cb_.clear();
}

Eigen::Isometry3d Tesseract::findTCP(const tesseract_planning::ManipulatorInfo& manip_info) const
{
  if (manip_info.tcp.empty())
    return Eigen::Isometry3d::Identity();

  auto composite_mi_fwd_kin = manipulator_manager_->getFwdKinematicSolver(manip_info.manipulator);
  if (composite_mi_fwd_kin == nullptr)
    throw std::runtime_error("findTCP: Manipulator '" + manip_info.manipulator + "' does not exist!");

  const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();
  if (manip_info.tcp.isString())
  {
    // Check Manipulator Manager for TCP
    const std::string& tcp_name = manip_info.tcp.getString();
    if (manipulator_manager_->hasGroupTCP(manip_info.manipulator, tcp_name))
      return manipulator_manager_->getGroupsTCP(manip_info.manipulator, tcp_name);

    // Check Environment for links and calculate TCP
    tesseract_environment::EnvState::ConstPtr env_state = environment_->getCurrentState();
    auto link_it = env_state->link_transforms.find(tcp_name);
    if (link_it != env_state->link_transforms.end())
      return env_state->link_transforms.at(tip_link).inverse() * link_it->second;

    // Check callbacks for TCP
    for (const auto& fn : find_tcp_cb_)
    {
      try
      {
        Eigen::Isometry3d tcp = fn(manip_info);
        return tcp;
      }
      catch (...)
      {
        CONSOLE_BRIDGE_logDebug("User Defined Find TCP Callback Failed!");
      }
    }

    throw std::runtime_error("Could not find tcp by name " + tcp_name + "' setting to Identity!");
  }

  if (manip_info.tcp.isTransform())
    return manip_info.tcp.getTransform();

  throw std::runtime_error("Could not find tcp!");
}

void Tesseract::addFindTCPCallback(FindTCPCallbackFn fn) { find_tcp_cb_.push_back(fn); }

}  // namespace tesseract
