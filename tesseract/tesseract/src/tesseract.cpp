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
#include <tesseract_environment/kdl/kdl_env.h>
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
Tesseract::Tesseract() : init_info_(new TesseractInitInfo()) { clear(); }

bool Tesseract::isInitialized() const { return initialized_; }

bool Tesseract::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph)
{
  clear();
  init_info_->type = TesseractInitType::SCENE_GRAPH;
  init_info_->scene_graph = scene_graph;

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(std::move(scene_graph)))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  srdf_model_ = std::make_shared<tesseract_scene_graph::SRDFModel>();
  srdf_model_->getName() = scene_graph->getName();
  srdf_model_const_ = srdf_model_;

  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph,
                     tesseract_scene_graph::SRDFModel::Ptr srdf_model)
{
  clear();
  init_info_->type = TesseractInitType::SCENE_GRAPH_SRDF_MODEL;
  init_info_->scene_graph = scene_graph;
  init_info_->srdf_model = srdf_model;

  srdf_model_ = std::move(srdf_model);
  srdf_model_const_ = srdf_model_;

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(std::move(scene_graph)))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;

  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const std::string& urdf_string, const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
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
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  srdf_model_ = std::make_shared<tesseract_scene_graph::SRDFModel>();
  srdf_model_->getName() = scene_graph->getName();
  srdf_model_const_ = srdf_model_;

  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const std::string& urdf_string,
                     const std::string& srdf_string,
                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
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
  srdf_model_ = srdf;
  srdf_model_const_ = srdf_model_;

  // Add allowed collision matrix to scene graph
  tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

  // Construct Environment
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const boost::filesystem::path& urdf_path,
                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
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
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  srdf_model_ = std::make_shared<tesseract_scene_graph::SRDFModel>();
  srdf_model_->getName() = scene_graph->getName();
  srdf_model_const_ = srdf_model_;

  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const boost::filesystem::path& urdf_path,
                     const boost::filesystem::path& srdf_path,
                     const tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();
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
  srdf_model_ = std::make_shared<tesseract_scene_graph::SRDFModel>();
  if (!srdf_model_->initFile(*scene_graph, srdf_path.string()))
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    srdf_model_ = nullptr;
    return false;
  }
  srdf_model_const_ = srdf_model_;

  // Add allowed collision matrix to scene graph
  tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf_model_);

  // Construct Environment
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const TesseractInitInfo::Ptr& init_info)
{
  switch (init_info->type)
  {
    case TesseractInitType::SCENE_GRAPH:
      init(init_info->scene_graph);
      break;
    case TesseractInitType::SCENE_GRAPH_SRDF_MODEL:
      init(init_info->scene_graph, init_info->srdf_model);
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
    default:
      CONSOLE_BRIDGE_logError("Unsupported TesseractInitInfo type.");
      return false;
  }
  return true;
}

const tesseract_environment::Environment::Ptr& Tesseract::getEnvironment() { return environment_; }

const tesseract_environment::Environment::ConstPtr& Tesseract::getEnvironmentConst() const
{
  return environment_const_;
}

const tesseract_scene_graph::SRDFModel::Ptr& Tesseract::getSRDFModel() const { return srdf_model_; }
const tesseract_scene_graph::SRDFModel::ConstPtr& Tesseract::getSRDFModelConst() const { return srdf_model_const_; }

tesseract_scene_graph::SRDFModel::GroupStates& Tesseract::getGroupStates() { return srdf_model_->getGroupStates(); }

const tesseract_scene_graph::SRDFModel::GroupStates& Tesseract::getGroupStatesConst() const
{
  return srdf_model_const_->getGroupStates();
}

tesseract_scene_graph::SRDFModel::GroupTCPs& Tesseract::getGroupTCPs() { return srdf_model_->getGroupTCPs(); }

const tesseract_scene_graph::SRDFModel::GroupTCPs& Tesseract::getGroupTCPs() const
{
  return srdf_model_->getGroupTCPs();
}

const ForwardKinematicsManager::Ptr& Tesseract::getFwdKinematicsManager() { return fwd_kin_manager_; }

const ForwardKinematicsManager::ConstPtr& Tesseract::getFwdKinematicsManagerConst() const
{
  return fwd_kin_manager_const_;
}

const InverseKinematicsManager::Ptr& Tesseract::getInvKinematicsManager() { return inv_kin_manager_; }

const InverseKinematicsManager::ConstPtr& Tesseract::getInvKinematicsManagerConst() const
{
  return inv_kin_manager_const_;
}

/** @brief registerDefaultContactManagers */
bool Tesseract::registerDefaultContactManagers()
{
  using namespace tesseract_collision;

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

bool Tesseract::registerDefaultFwdKinSolvers()
{
  bool success = true;

  fwd_kin_manager_ = std::make_shared<ForwardKinematicsManager>();
  fwd_kin_manager_const_ = fwd_kin_manager_;

  auto chain_factory = std::make_shared<tesseract_kinematics::KDLFwdKinChainFactory>();
  fwd_kin_manager_->registerFwdKinematicsFactory(chain_factory);

  auto tree_factory = std::make_shared<tesseract_kinematics::KDLFwdKinTreeFactory>();
  fwd_kin_manager_->registerFwdKinematicsFactory(tree_factory);

  for (const auto& group : srdf_model_->getChainGroups())
  {
    if (!group.second.empty())
    {
      tesseract_kinematics::ForwardKinematics::Ptr solver =
          chain_factory->create(environment_->getSceneGraph(), group.second, group.first);
      if (solver != nullptr)
      {
        if (!fwd_kin_manager_->addFwdKinematicSolver(solver))
        {
          CONSOLE_BRIDGE_logError("Failed to add inverse kinematic chain solver %s for manipulator %s to manager!",
                                  solver->getSolverName().c_str(),
                                  group.first.c_str());
          success = false;
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Failed to create inverse kinematic chain solver %s for manipulator %s!",
                                solver->getSolverName().c_str(),
                                group.first.c_str());
        success = false;
      }
    }
  }

  for (const auto& group : srdf_model_->getJointGroups())
  {
    if (!group.second.empty())
    {
      tesseract_kinematics::ForwardKinematics::Ptr solver =
          tree_factory->create(environment_->getSceneGraph(), group.second, group.first);
      if (solver != nullptr)
      {
        if (!fwd_kin_manager_->addFwdKinematicSolver(solver))
        {
          CONSOLE_BRIDGE_logError("Failed to add inverse kinematic tree solver %s for manipulator %s to manager!",
                                  solver->getSolverName().c_str(),
                                  group.first.c_str());
          success = false;
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Failed to create inverse kinematic tree solver %s for manipulator %s!",
                                solver->getSolverName().c_str(),
                                group.first.c_str());
        success = false;
      }
    }
  }

  for (const auto& group : srdf_model_->getLinkGroups())
  {
    // TODO: Need to add other options
    if (!group.second.empty())
    {
      CONSOLE_BRIDGE_logError("Link groups are currently not supported!");
      success = false;
    }
  }

  return success;
}

bool Tesseract::registerDefaultInvKinSolvers()
{
  bool success = true;

  inv_kin_manager_ = std::make_shared<InverseKinematicsManager>();
  inv_kin_manager_const_ = inv_kin_manager_;

  auto factory = std::make_shared<tesseract_kinematics::KDLInvKinChainLMAFactory>();
  inv_kin_manager_->registerInvKinematicsFactory(factory);

  for (const auto& group : srdf_model_->getChainGroups())
  {
    if (!group.second.empty())
    {
      tesseract_kinematics::InverseKinematics::Ptr solver =
          factory->create(environment_->getSceneGraph(), group.second, group.first);
      if (solver != nullptr)
      {
        if (!inv_kin_manager_->addInvKinematicSolver(solver))
        {
          CONSOLE_BRIDGE_logError("Failed to add inverse kinematic chain solver %s for manipulator %s to manager!",
                                  solver->getSolverName().c_str(),
                                  group.first.c_str());
          success = false;
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Failed to create inverse kinematic chain solver %s for manipulator %s!",
                                solver->getSolverName().c_str(),
                                group.first.c_str());
        success = false;
      }
    }
  }

  for (const auto& group : srdf_model_->getJointGroups())
  {
    if (!group.second.empty())
    {
      CONSOLE_BRIDGE_logError("Joint groups are currently not supported by inverse kinematics!");
      success = false;
    }
  }

  for (const auto& group : srdf_model_->getLinkGroups())
  {
    if (!group.second.empty())
    {
      CONSOLE_BRIDGE_logError("Link groups are currently not supported by inverse kinematics!");
      success = false;
    }
  }

  for (const auto& group : srdf_model_->getGroupOPWKinematics())
  {
    if (!group.first.empty())
    {
      tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = fwd_kin_manager_->getFwdKinematicSolver(group.first);
      if (fwd_kin == nullptr)
      {
        CONSOLE_BRIDGE_logError("Failed to add inverse kinematic opw solver for manipulator %s to manager!",
                                group.first.c_str());
        success = false;
      }
      else
      {
        opw_kinematics::Parameters<double> params;
        params.a1 = group.second.a1;
        params.a2 = group.second.a2;
        params.b = group.second.b;
        params.c1 = group.second.c1;
        params.c2 = group.second.c2;
        params.c3 = group.second.c3;
        params.c4 = group.second.c4;
        for (std::size_t i = 0; i < 6; ++i)
        {
          params.offsets[i] = group.second.offsets[i];
          params.sign_corrections[i] = group.second.sign_corrections[i];
        }

        auto solver = std::make_shared<tesseract_kinematics::OPWInvKin>();
        solver->init(group.first,
                     params,
                     fwd_kin->getBaseLinkName(),
                     fwd_kin->getTipLinkName(),
                     fwd_kin->getJointNames(),
                     fwd_kin->getLinkNames(),
                     fwd_kin->getActiveLinkNames(),
                     fwd_kin->getLimits());

        if (solver->checkInitialized())
        {
          if (!inv_kin_manager_->addInvKinematicSolver(solver))
          {
            CONSOLE_BRIDGE_logError("Failed to add inverse kinematic opw solver for manipulator %s to manager!",
                                    group.first.c_str());
            success = false;
          }
        }
        else
        {
          CONSOLE_BRIDGE_logError("Failed to create inverse kinematic opw solver for manipulator %s!",
                                  group.first.c_str());
          success = false;
        }
      }
    }
  }

  return success;
}

void Tesseract::clear()
{
  initialized_ = false;
  environment_ = nullptr;
  environment_const_ = nullptr;
  srdf_model_ = nullptr;
  srdf_model_const_ = nullptr;
  inv_kin_manager_ = nullptr;
  inv_kin_manager_const_ = nullptr;
  fwd_kin_manager_ = nullptr;
  fwd_kin_manager_const_ = nullptr;
}
}  // namespace tesseract
