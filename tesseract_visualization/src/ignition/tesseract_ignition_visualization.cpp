/**
 * @file tesseract_ignition_vizualization.cpp
 * @brief A tesseract vizualization implementation leveraging Ignition Robotics
 *
 * @author Levi Armstrong
 * @date May 14, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <ignition/msgs/MessageTypes.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <chrono>
#include <numeric>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/ignition/tesseract_ignition_visualization.h>
#include <tesseract_visualization/ignition/conversions.h>
#include <tesseract_visualization/trajectory_player.h>
#include <tesseract_visualization/markers/arrow_marker.h>
#include <tesseract_visualization/markers/axis_marker.h>
#include <tesseract_visualization/markers/contact_results_marker.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_common/class_loader.h>

/** @brief Message type is ignition::msgs::Scene */
static const std::string DEFAULT_SCENE_TOPIC_NAME = "/tesseract_ignition/topic/scene";

/** @brief Message type is ignition::msgs::Pose_V */
static const std::string DEFAULT_POSE_TOPIC_NAME = "/tesseract_ignition/topic/pose";

/** @brief Message type is ignition::msgs::UInt32_V */
static const std::string DEFAULT_DELETION_TOPIC_NAME = "/tesseract_ignition/topic/deletion";

static const std::string COLLISION_RESULTS_MODEL_NAME = "tesseract_collision_results_model";
static const std::string AXES_MODEL_NAME = "tesseract_axes_model";
static const std::string ARROW_MODEL_NAME = "tesseract_arrow_model";
static const std::string TOOL_PATH_MODEL_NAME = "tesseract_tool_path_model";

namespace tesseract_visualization
{
TesseractIgnitionVisualization::TesseractIgnitionVisualization()
{
  scene_pub_ = node_.Advertise<ignition::msgs::Scene>(DEFAULT_SCENE_TOPIC_NAME);
  pose_pub_ = node_.Advertise<ignition::msgs::Pose_V>(DEFAULT_POSE_TOPIC_NAME);
  deletion_pub_ = node_.Advertise<ignition::msgs::UInt32_V>(DEFAULT_DELETION_TOPIC_NAME);
}

bool TesseractIgnitionVisualization::isConnected() const
{
  return scene_pub_.HasConnections() && pose_pub_.HasConnections() && deletion_pub_.HasConnections();
}

void TesseractIgnitionVisualization::waitForConnection(long seconds) const
{
  if (seconds == 0)
    seconds = std::numeric_limits<long>::max();

  for (int i = 0; i < seconds; ++i)
  {
    if (!isConnected())
      sleep(1);
    else
      break;
  }
}

void TesseractIgnitionVisualization::plotEnvironment(const tesseract_environment::Environment& env, std::string /*ns*/)
{
  ignition::msgs::Scene msg;
  toMsg(msg, entity_manager_, *(env.getSceneGraph()), env.getState().link_transforms);
  scene_pub_.Publish(msg);
}

void TesseractIgnitionVisualization::plotEnvironmentState(const tesseract_scene_graph::SceneState& state,
                                                          std::string /*ns*/)
{
  sendSceneState(state);
}

void TesseractIgnitionVisualization::plotTrajectory(const tesseract_common::JointTrajectory& traj,
                                                    const tesseract_scene_graph::StateSolver& state_solver,
                                                    std::string /*ns*/)
{
  std::chrono::duration<double> fp_s(5.0 / static_cast<double>(traj.size()));
  for (const auto& traj_state : traj)
  {
    tesseract_scene_graph::SceneState state = state_solver.getState(traj_state.joint_names, traj_state.position);
    sendSceneState(state);
    std::this_thread::sleep_for(fp_s);
  }
}

void addArrow(EntityManager& entity_manager, ignition::msgs::Link& link, long& sub_index, const ArrowMarker& marker)
{
  std::string gv_name = link.name() + "_" + std::to_string(++sub_index);
  ignition::msgs::Visual* gv_msg = link.add_visual();
  gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
  gv_msg->set_name(gv_name);

  gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(marker.pose)));

  ignition::msgs::Geometry geometry_msg;
  geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_CYLINDER);
  ignition::msgs::CylinderGeom shape_geometry_msg;
  shape_geometry_msg.set_radius(marker.shaft_radius);
  shape_geometry_msg.set_length(marker.shaft_length);
  geometry_msg.mutable_cylinder()->CopyFrom(shape_geometry_msg);
  gv_msg->mutable_geometry()->CopyFrom(geometry_msg);
  ignition::msgs::Material shape_material_msg;
  shape_material_msg.mutable_diffuse()->set_r(static_cast<float>(marker.material->color(0)));
  shape_material_msg.mutable_diffuse()->set_g(static_cast<float>(marker.material->color(1)));
  shape_material_msg.mutable_diffuse()->set_b(static_cast<float>(marker.material->color(2)));
  shape_material_msg.mutable_diffuse()->set_a(static_cast<float>(marker.material->color(3)));
  gv_msg->mutable_material()->CopyFrom(shape_material_msg);
  gv_msg->set_parent_name(link.name());
}

void addCylinder(EntityManager& entity_manager,
                 ignition::msgs::Link& link,
                 long& sub_index,
                 const Eigen::Ref<const Eigen::Vector3d>& pt1,
                 const Eigen::Ref<const Eigen::Vector3d>& pt2,
                 const tesseract_scene_graph::Material& material,
                 const Eigen::Vector3d& /*scale*/)
{
  std::string gv_name = link.name() + "_" + std::to_string(++sub_index);
  ignition::msgs::Visual* gv_msg = link.add_visual();
  gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
  gv_msg->set_name(gv_name);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d x, y, z;
  z = (pt2 - pt1).normalized();
  y = z.unitOrthogonal();
  x = (y.cross(z)).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  pose.linear() = rot;
  pose.translation() = pt1 + (((pt2 - pt1).norm() / 2) * z);

  gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(pose)));

  ignition::msgs::Geometry geometry_msg;
  geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_CYLINDER);
  ignition::msgs::CylinderGeom shape_geometry_msg;
  shape_geometry_msg.set_radius(1); /** @todo LEVI fix */
  shape_geometry_msg.set_length((pt2 - pt1).norm());
  geometry_msg.mutable_cylinder()->CopyFrom(shape_geometry_msg);
  gv_msg->mutable_geometry()->CopyFrom(geometry_msg);
  ignition::msgs::Material shape_material_msg;
  shape_material_msg.mutable_diffuse()->set_r(static_cast<float>(material.color(0)));
  shape_material_msg.mutable_diffuse()->set_g(static_cast<float>(material.color(1)));
  shape_material_msg.mutable_diffuse()->set_b(static_cast<float>(material.color(2)));
  shape_material_msg.mutable_diffuse()->set_a(static_cast<float>(material.color(3)));
  gv_msg->mutable_material()->CopyFrom(shape_material_msg);
  gv_msg->set_parent_name(link.name());
}

void addAxis(EntityManager& entity_manager, ignition::msgs::Link& link, long& sub_index, const AxisMarker& m)
{
  Eigen::Vector3d x_axis = m.axis.matrix().block<3, 1>(0, 0);
  Eigen::Vector3d y_axis = m.axis.matrix().block<3, 1>(0, 1);
  Eigen::Vector3d z_axis = m.axis.matrix().block<3, 1>(0, 2);
  Eigen::Vector3d position = m.axis.matrix().block<3, 1>(0, 3);
  Eigen::Vector3d scale = m.getScale();

  std::string gv_name = link.name() + "_" + std::to_string(++sub_index);
  tesseract_scene_graph::Material axis_red("axis_red");
  axis_red.color = Eigen::Vector4d(1, 0, 0, 1);
  tesseract_scene_graph::Material axis_green("axis_green");
  axis_red.color = Eigen::Vector4d(0, 1, 0, 1);
  tesseract_scene_graph::Material axis_blue("axis_blue");
  axis_red.color = Eigen::Vector4d(0, 0, 1, 1);

  addCylinder(entity_manager, link, sub_index, position, position + (scale(0) * x_axis), axis_red, scale * (1.0 / 20));
  addCylinder(
      entity_manager, link, sub_index, position, position + (scale(1) * y_axis), axis_green, scale * (1.0 / 20));
  addCylinder(entity_manager, link, sub_index, position, position + (scale(2) * z_axis), axis_blue, scale * (1.0 / 20));
}

void TesseractIgnitionVisualization::plotMarker(const Marker& marker, std::string /*ns*/)
{
  switch (marker.getType())
  {
    case static_cast<int>(MarkerType::ARROW):
    {
      const auto& m = dynamic_cast<const ArrowMarker&>(marker);
      ignition::msgs::Scene scene_msg;
      scene_msg.set_name("scene");
      ignition::msgs::Model* model = scene_msg.add_model();
      std::string model_name = ARROW_MODEL_NAME;
      model->set_name(model_name);
      model->set_id(static_cast<unsigned>(entity_manager_.addModel(model_name)));

      long cnt = 0;
      std::string link_name = model_name + std::to_string(++cnt);
      ignition::msgs::Link* link_msg = model->add_link();
      link_msg->set_id(static_cast<unsigned>(entity_manager_.addVisual(link_name)));
      link_msg->set_name(link_name);
      addArrow(entity_manager_, *link_msg, cnt, m);
      scene_pub_.Publish(scene_msg);
      break;
    }
    case static_cast<int>(MarkerType::AXIS):
    {
      const auto& m = dynamic_cast<const AxisMarker&>(marker);

      ignition::msgs::Scene scene_msg;
      scene_msg.set_name("scene");
      ignition::msgs::Model* model = scene_msg.add_model();
      std::string model_name = AXES_MODEL_NAME;
      model->set_name(model_name);
      model->set_id(static_cast<unsigned>(entity_manager_.addModel(model_name)));

      long cnt = 0;
      std::string link_name = model_name + std::to_string(++cnt);
      ignition::msgs::Link* link_msg = model->add_link();
      link_msg->set_id(static_cast<unsigned>(entity_manager_.addVisual(link_name)));
      link_msg->set_name(link_name);
      addAxis(entity_manager_, *link_msg, cnt, m.axis);
      scene_pub_.Publish(scene_msg);
      break;
    }
    case static_cast<int>(MarkerType::CONTACT_RESULTS):
    {
      const auto& m = dynamic_cast<const ContactResultsMarker&>(marker);

      ignition::msgs::Scene scene_msg;
      scene_msg.set_name("scene");
      ignition::msgs::Model* model = scene_msg.add_model();
      std::string model_name = COLLISION_RESULTS_MODEL_NAME;
      model->set_name(model_name);
      model->set_id(static_cast<unsigned>(entity_manager_.addModel(model_name)));

      long cnt = 0;
      for (size_t i = 0; i < m.dist_results.size(); ++i)
      {
        const tesseract_collision::ContactResult& dist = m.dist_results[i];
        double safety_distance = m.margin_data.getPairCollisionMargin(dist.link_names[0], dist.link_names[1]);

        std::string link_name = model_name + std::to_string(++cnt);
        ignition::msgs::Link* link_msg = model->add_link();
        link_msg->set_id(static_cast<unsigned>(entity_manager_.addVisual(link_name)));
        link_msg->set_name(link_name);

        auto base_material = std::make_shared<tesseract_scene_graph::Material>("base_material");
        if (dist.distance < 0)
        {
          base_material->color << 1.0, 0.0, 0.0, 1.0;
        }
        else if (dist.distance < safety_distance)
        {
          base_material->color << 1.0, 1.0, 0.0, 1.0;
        }
        else
        {
          base_material->color << 0.0, 1.0, 0.0, 1.0;
        }

        if (dist.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Between)
        {
          ArrowMarker am(dist.transform[0] * dist.nearest_points_local[0],
                         dist.cc_transform[0] * dist.nearest_points_local[0]);
          am.material = std::make_shared<tesseract_scene_graph::Material>("cc_material");
          am.material->color << 0.0, 0.0, 1.0, 1.0;
          addArrow(entity_manager_, *link_msg, cnt, am);
        }

        if (dist.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Between)
        {
          ArrowMarker am(dist.transform[1] * dist.nearest_points_local[1],
                         dist.cc_transform[1] * dist.nearest_points_local[1]);
          am.material = std::make_shared<tesseract_scene_graph::Material>("cc_material");
          am.material->color << 0.0, 0.0, 0.5, 1.0;
          addArrow(entity_manager_, *link_msg, cnt, am);
        }

        auto it0 = std::find(m.link_names.begin(), m.link_names.end(), dist.link_names[0]);
        auto it1 = std::find(m.link_names.begin(), m.link_names.end(), dist.link_names[1]);

        if (it0 != m.link_names.end() && it1 != m.link_names.end())
        {
          ArrowMarker am1(dist.nearest_points[0], dist.nearest_points[1]);
          am1.material = base_material;
          addArrow(entity_manager_, *link_msg, cnt, am1);

          ArrowMarker am2(dist.nearest_points[1], dist.nearest_points[0]);
          am2.material = base_material;
          addArrow(entity_manager_, *link_msg, cnt, am2);
        }
        else if (it0 != m.link_names.end())
        {
          ArrowMarker am(dist.nearest_points[1], dist.nearest_points[0]);
          am.material = base_material;
          addArrow(entity_manager_, *link_msg, cnt, am);
        }
        else
        {
          ArrowMarker am(dist.nearest_points[0], dist.nearest_points[1]);
          am.material = base_material;
          addArrow(entity_manager_, *link_msg, cnt, am);
        }
      }
      scene_pub_.Publish(scene_msg);
      break;
    }
    default:
    {
      ignwarn << "plotMarkers: Unsupported marker type: " << std::to_string(marker.getType()) << std::endl;
    }
  }
}

void TesseractIgnitionVisualization::plotMarkers(const std::vector<Marker::Ptr>& /*markers*/, std::string /*ns*/)
{
  ignerr << "plotMarkers is currently not implemented!" << std::endl;
}

void TesseractIgnitionVisualization::clear(std::string /*ns*/)
{
  ignition::msgs::UInt32_V deletion_msg;
  long id = entity_manager_.getModel(COLLISION_RESULTS_MODEL_NAME);
  if (id >= 1000)
    deletion_msg.add_data(static_cast<unsigned>(id));

  id = entity_manager_.getModel(ARROW_MODEL_NAME);
  if (id >= 1000)
    deletion_msg.add_data(static_cast<unsigned>(id));

  id = entity_manager_.getModel(AXES_MODEL_NAME);
  if (id >= 1000)
    deletion_msg.add_data(static_cast<unsigned>(id));

  deletion_pub_.Publish(deletion_msg);
}

void TesseractIgnitionVisualization::waitForInput(std::string message)
{
  std::cout << message << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void TesseractIgnitionVisualization::sendSceneState(const tesseract_scene_graph::SceneState& scene_state)
{
  ignition::msgs::Pose_V pose_v;
  for (const auto& pair : scene_state.link_transforms)
  {
    ignition::msgs::Pose* pose = pose_v.add_pose();
    pose->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(pair.second)));
    pose->set_name(pair.first);
    pose->set_id(static_cast<unsigned>(entity_manager_.getLink(pair.first)));
  }

  if (!pose_pub_.Publish(pose_v))
  {
    ignerr << "Failed to publish pose vector!" << std::endl;
  }
}

// void TesseractIgnitionVisualization::plotTrajectory(const tesseract_planning::Instruction& instruction)
//{
//  using namespace tesseract_planning;
//  tesseract_environment::StateSolver::Ptr state_solver = env_->getStateSolver();

//  CompositeInstruction c;
//  if (isCompositeInstruction(instruction))
//  {
//    c = *(instruction.cast_const<CompositeInstruction>());
//  }
//  else if (isMoveInstruction(instruction))
//  {
//    const auto* mi = instruction.cast_const<MoveInstruction>();
//    c.setManipulatorInfo(mi->getManipulatorInfo());
//    c.push_back(instruction);
//  }
//  else
//  {
//    ignerr << "plotTrajectoy: Unsupported Instruction Type!" << std::endl;
//    return;
//  }

//  TrajectoryPlayer player;
//  player.setProgram(c);
//  while (!player.isFinished())
//  {
//    tesseract_planning::MoveInstruction mi = player.getNext();
//    if (isStateWaypoint(mi.getWaypoint()))
//    {
//      const auto* swp = mi.getWaypoint().cast_const<StateWaypoint>();
//      assert(static_cast<long>(swp->joint_names.size()) == swp->position.size());
//      tesseract_environment::EnvState::Ptr state = state_solver->getState(swp->joint_names, swp->position);
//      sendEnvState(state);
//    }
//    else if (isJointWaypoint(mi.getWaypoint()))
//    {
//      const auto* jwp = mi.getWaypoint().cast_const<JointWaypoint>();
//      assert(static_cast<long>(jwp->joint_names.size()) == jwp->size());
//      tesseract_environment::EnvState::Ptr state = state_solver->getState(jwp->joint_names, *jwp);
//      sendEnvState(state);
//    }
//    else
//    {
//      ignerr << "plotTrajectoy: Unsupported Waypoint Type!" << std::endl;
//    }
//  }
//}

// void TesseractIgnitionVisualization::plotToolPath(const tesseract_planning::Instruction& instruction)
//{
//  using namespace tesseract_planning;

//  ignition::msgs::Scene scene_msg;
//  scene_msg.set_name("scene");
//  ignition::msgs::Model* model = scene_msg.add_model();
//  std::string model_name = TOOL_PATH_MODEL_NAME;
//  model->set_name(model_name);
//  model->set_id(static_cast<unsigned>(entity_manager_.addModel(model_name)));

//  tesseract_environment::StateSolver::Ptr state_solver = env_->getStateSolver();
//  if (isCompositeInstruction(instruction))
//  {
//    const auto* ci = instruction.cast_const<CompositeInstruction>();

//    // Assume all the plan instructions have the same manipulator as the composite
//    assert(!ci->getManipulatorInfo().empty());
//    const tesseract_common::ManipulatorInfo& composite_mi = ci->getManipulatorInfo();

//    auto composite_mi_fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(composite_mi.manipulator);
//    if (composite_mi_fwd_kin == nullptr)
//    {
//      ignerr << "plotToolPath: Manipulator: " << composite_mi.manipulator << " does not exist!" << std::endl;
//      return;
//    }
//    const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();

//    std::vector<std::reference_wrapper<const Instruction>> fi = tesseract_planning::flatten(*ci, planFilter);
//    if (fi.empty())
//      fi = tesseract_planning::flatten(*ci, moveFilter);

//    long cnt = 0;
//    for (const auto& i : fi)
//    {
//      tesseract_common::ManipulatorInfo manip_info;

//      std::string link_name = model_name + std::to_string(++cnt);
//      ignition::msgs::Link* link_msg = model->add_link();
//      link_msg->set_id(static_cast<unsigned>(entity_manager_.addVisual(link_name)));
//      link_msg->set_name(link_name);

//      // Check for updated manipulator information and get waypoint
//      Waypoint wp;
//      if (isPlanInstruction(i.get()))
//      {
//        const auto* pi = i.get().cast_const<PlanInstruction>();
//        manip_info = composite_mi.getCombined(pi->getManipulatorInfo());
//        wp = pi->getWaypoint();
//      }
//      else if (isMoveInstruction(i.get()))
//      {
//        const auto* mi = i.get().cast_const<MoveInstruction>();
//        manip_info = composite_mi.getCombined(mi->getManipulatorInfo());
//        wp = mi->getWaypoint();
//      }

//      // Extract TCP
//      Eigen::Isometry3d tcp = env_->findTCP(manip_info);

//      if (isStateWaypoint(wp))
//      {
//        const auto* swp = wp.cast_const<StateWaypoint>();
//        assert(static_cast<long>(swp->joint_names.size()) == swp->position.size());
//        tesseract_environment::EnvState::Ptr state = state_solver->getState(swp->joint_names, swp->position);
//        addAxis(entity_manager_, *link_msg, cnt, link_name, state->link_transforms[tip_link] * tcp, 0.05);
//      }
//      else if (isJointWaypoint(wp))
//      {
//        const auto* jwp = wp.cast_const<JointWaypoint>();
//        assert(static_cast<long>(jwp->joint_names.size()) == jwp->size());
//        tesseract_environment::EnvState::Ptr state = state_solver->getState(jwp->joint_names, *jwp);
//        addAxis(entity_manager_, *link_msg, cnt, link_name, state->link_transforms[tip_link] * tcp, 0.05);
//      }
//      else if (isCartesianWaypoint(wp))
//      {
//        const auto* cwp = wp.cast_const<CartesianWaypoint>();
//        if (manip_info.working_frame.empty())
//        {
//          addAxis(entity_manager_, *link_msg, cnt, link_name, (*cwp), 0.05);
//        }
//        else
//        {
//          tesseract_environment::EnvState::ConstPtr state = env_->getCurrentState();
//          addAxis(entity_manager_,
//                  *link_msg,
//                  cnt,
//                  link_name,
//                  state->link_transforms.at(manip_info.working_frame) * (*cwp),
//                  0.05);
//        }
//      }
//      else
//      {
//        ignerr << "plotTrajectoy: Unsupported Waypoint Type!" << std::endl;
//      }
//    }
//  }
//  else if (isPlanInstruction(instruction))
//  {
//    long cnt = 0;
//    std::string link_name = model_name + std::to_string(++cnt);
//    ignition::msgs::Link* link_msg = model->add_link();
//    link_msg->set_id(static_cast<unsigned>(entity_manager_.addVisual(link_name)));
//    link_msg->set_name(link_name);

//    assert(isPlanInstruction(instruction));
//    const auto* pi = instruction.cast_const<PlanInstruction>();

//    // Assume all the plan instructions have the same manipulator as the composite
//    assert(!pi->getManipulatorInfo().empty());
//    const tesseract_common::ManipulatorInfo& composite_mi = pi->getManipulatorInfo();
//    tesseract_common::ManipulatorInfo manip_info = composite_mi.getCombined(pi->getManipulatorInfo());

//    auto composite_mi_fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
//    if (composite_mi_fwd_kin == nullptr)
//    {
//      ignerr << "plotToolPath: Manipulator: " << manip_info.manipulator << " does not exist!" << std::endl;
//      return;
//    }
//    const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();

//    // Extract TCP
//    Eigen::Isometry3d tcp = env_->findTCP(manip_info);

//    if (isStateWaypoint(pi->getWaypoint()))
//    {
//      const auto* swp = pi->getWaypoint().cast_const<StateWaypoint>();
//      assert(static_cast<long>(swp->joint_names.size()) == swp->position.size());
//      tesseract_environment::EnvState::Ptr state = state_solver->getState(swp->joint_names, swp->position);
//      addAxis(entity_manager_, *link_msg, cnt, link_name, state->link_transforms[tip_link] * tcp, 0.05);
//    }
//    else if (isJointWaypoint(pi->getWaypoint()))
//    {
//      const auto* jwp = pi->getWaypoint().cast_const<JointWaypoint>();
//      assert(static_cast<long>(jwp->joint_names.size()) == jwp->size());
//      tesseract_environment::EnvState::Ptr state = state_solver->getState(jwp->joint_names, *jwp);
//      addAxis(entity_manager_, *link_msg, cnt, link_name, state->link_transforms[tip_link] * tcp, 0.05);
//    }
//    else if (isCartesianWaypoint(pi->getWaypoint()))
//    {
//      const auto* cwp = pi->getWaypoint().cast_const<CartesianWaypoint>();
//      if (manip_info.working_frame.empty())
//      {
//        addAxis(entity_manager_, *link_msg, cnt, link_name, (*cwp), 0.05);
//      }
//      else
//      {
//        tesseract_environment::EnvState::ConstPtr state = env_->getCurrentState();
//        addAxis(entity_manager_,
//                *link_msg,
//                cnt,
//                link_name,
//                state->link_transforms.at(manip_info.working_frame) * (*cwp),
//                0.05);
//      }
//    }
//    else
//    {
//      ignerr << "plotTrajectoy: Unsupported Waypoint Type!" << std::endl;
//    }
//  }
//  else
//  {
//    ignerr << "plotTrajectoy: Unsupported Instruction Type!" << std::endl;
//  }

//  scene_pub_.Publish(scene_msg);
//}

TESSERACT_PLUGIN_ANCHOR_IMPL(IgnitionVisualizationAnchor)
}  // namespace tesseract_visualization

TESSERACT_ADD_VISUALIZATION_PLUGIN(tesseract_visualization::TesseractIgnitionVisualization,
                                   TesseractIgnitionVisualizationPlugin)
