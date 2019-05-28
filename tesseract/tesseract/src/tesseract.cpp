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
#include <tesseract/tesseract.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>

namespace tesseract
{
  Tesseract::Tesseract()
  {
    clear();
  }

  bool Tesseract::isInitialized() const
  {
    return initialized_;
  }

  bool Tesseract::init(tesseract_scene_graph::SceneGraphPtr scene_graph)
  {
    clear();

    // Construct Environment from Scene Graph
    environment_ = std::make_shared<tesseract_environment::KDLEnv>();
    if (!environment_->init(scene_graph))
    {
      CONSOLE_BRIDGE_logError("Failed to initialize environment.");
      return false;
    }
    environment_const_ = environment_;
    registerDefaultContactManagers();

    initialized_ = true;
    return true;
  }

  bool Tesseract::init(tesseract_scene_graph::SceneGraphPtr scene_graph, tesseract_scene_graph::SRDFModelConstPtr srdf_model)
  {
    clear();

    srdf_model_ = srdf_model;

    // Construct Environment from Scene Graph
    environment_ = std::make_shared<tesseract_environment::KDLEnv>();
    if (!environment_->init(scene_graph))
    {
      CONSOLE_BRIDGE_logError("Failed to initialize environment.");
      return false;
    }
    environment_const_ = environment_;
    registerDefaultContactManagers();

    // Create kinematics map from srdf
    fwd_kin_map_ = tesseract_kinematics::createKinematicsMap<tesseract_kinematics::KDLFwdKinChain, tesseract_kinematics::KDLFwdKinTree>(scene_graph, *srdf_model_);

    initialized_ = true;
    return true;
  }

  bool Tesseract::init(const std::string& urdf_string, tesseract_scene_graph::ResourceLocatorFn locator)
  {
    clear();

    // Parse urdf string into Scene Graph
    tesseract_scene_graph::SceneGraphPtr scene_graph = tesseract_scene_graph::parseURDFString(urdf_string, locator);
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
    registerDefaultContactManagers();

    initialized_ = true;
    return true;
  }

  bool Tesseract::init(const std::string& urdf_string, const std::string& srdf_string, tesseract_scene_graph::ResourceLocatorFn locator)
  {
    clear();

    // Parse urdf string into Scene Graph
    tesseract_scene_graph::SceneGraphPtr scene_graph = tesseract_scene_graph::parseURDFString(urdf_string, locator);
    if (scene_graph == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to parse URDF.");
      return false;
    }

    // Parse srdf string into SRDF Model
    tesseract_scene_graph::SRDFModelPtr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
    if (!srdf->initString(*scene_graph, srdf_string))
    {
      CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
      return false;
    }
    srdf_model_ = srdf;

    // Add allowed collision matrix to scene graph
    tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

    // Create kinematics map from srdf
    fwd_kin_map_ = tesseract_kinematics::createKinematicsMap<tesseract_kinematics::KDLFwdKinChain, tesseract_kinematics::KDLFwdKinTree>(scene_graph, *srdf_model_);

    // Construct Environment
    environment_ = std::make_shared<tesseract_environment::KDLEnv>();
    if (!environment_->init(scene_graph))
    {
      CONSOLE_BRIDGE_logError("Failed to initialize environment.");
      return false;
    }
    environment_const_ = environment_;
    registerDefaultContactManagers();

    initialized_ = true;
    return true;
  }

  bool Tesseract::init(const boost::filesystem::path& urdf_path, tesseract_scene_graph::ResourceLocatorFn locator)
  {
    clear();

    // Parse urdf file into Scene Graph
    tesseract_scene_graph::SceneGraphPtr scene_graph = tesseract_scene_graph::parseURDFFile(urdf_path.string(), locator);
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
    registerDefaultContactManagers();

    initialized_ = true;
    return true;
  }

  bool Tesseract::init(const boost::filesystem::path& urdf_path, const boost::filesystem::path& srdf_path, tesseract_scene_graph::ResourceLocatorFn locator)
  {
    clear();

    // Parse urdf file into Scene Graph
    tesseract_scene_graph::SceneGraphPtr scene_graph = tesseract_scene_graph::parseURDFFile(urdf_path.string(), locator);
    if (scene_graph == nullptr)
    {
      CONSOLE_BRIDGE_logError("Failed to parse URDF.");
      return false;
    }

    // Parse srdf file into SRDF Model
    tesseract_scene_graph::SRDFModelPtr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
    if (!srdf->initFile(*scene_graph, srdf_path.string()))
    {
      CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
      return false;
    }
    srdf_model_ = srdf;

    // Add allowed collision matrix to scene graph
    tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

    // Create kinematics map from srdf
    fwd_kin_map_ = tesseract_kinematics::createKinematicsMap<tesseract_kinematics::KDLFwdKinChain, tesseract_kinematics::KDLFwdKinTree>(scene_graph, *srdf_model_);

    // Construct Environment
    environment_ = std::make_shared<tesseract_environment::KDLEnv>();
    if (!environment_->init(scene_graph))
    {
      CONSOLE_BRIDGE_logError("Failed to initialize environment.");
      return false;
    }
    environment_const_ = environment_;
    registerDefaultContactManagers();

    initialized_ = true;
    return true;
  }

  const tesseract_environment::EnvironmentPtr& Tesseract::getEnvironment()
  {
    return environment_;
  }

  const tesseract_environment::EnvironmentConstPtr& Tesseract::getEnvironmentConst() const
  {
    return environment_const_;
  }

  const tesseract_scene_graph::SRDFModelConstPtr& Tesseract::getSRDFModel() const
  {
    return srdf_model_;
  }

  const tesseract_kinematics::ForwardKinematicsConstPtrMap& Tesseract::getFwdKinematics()
  {
    return fwd_kin_map_;
  }

  tesseract_kinematics::ForwardKinematicsConstPtr Tesseract::getFwdKinematics(const std::string& name)
  {
    auto it = fwd_kin_map_.find(name);
    if (it == fwd_kin_map_.end())
    {
      return nullptr;
    }
    else
    {
      return it->second;
    }
  }

  /** @brief registerDefaultContactManagers */
  bool Tesseract::registerDefaultContactManagers()
  {
    using namespace tesseract_collision;

    // Register contact manager
    environment_->registerDiscreteContactManager("bullet", &tesseract_collision_bullet::BulletDiscreteBVHManager::create);
    environment_->registerContinuousContactManager("bullet", &tesseract_collision_bullet::BulletCastBVHManager::create);

    // Set Active contact manager
    environment_->setActiveDiscreteContactManager("bullet");
    environment_->setActiveContinuousContactManager("bullet");

    return true;
  }

  void Tesseract::clear()
  {
    initialized_ = false;
    environment_ = nullptr;
    environment_const_ = nullptr;
    srdf_model_ = nullptr;
    fwd_kin_map_.clear();
  }
}
