/**
 * @file conversions.cpp
 * @brief A set of conversion between Tesseract and Ignition Robotics objects
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
#include <ignition/msgs/geometry.pb.h>
#include <ignition/msgs/material.pb.h>
#include <ignition/msgs/inertial.pb.h>
#include <ignition/msgs/collision.pb.h>
#include <ignition/msgs/visual.pb.h>
#include <ignition/msgs/link.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/ignition/conversions.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_visualization
{
bool isMeshWithColor(const std::string& file_path)
{
  if (file_path.length() >= 4)
  {
    std::string last_four = file_path.substr(file_path.length() - 4);
    std::string last_four_lower;
    last_four_lower.resize(4);
    std::transform(last_four.begin(), last_four.end(), last_four_lower.begin(), ::tolower);
    return (last_four_lower == ".dae") || (last_four_lower == ".obj");
  }

  return false;
}

ignition::msgs::Material convert(const Eigen::Vector4d& rgba)
{
  ignition::msgs::Material shape_material_msg;
  shape_material_msg.mutable_ambient()->set_r(static_cast<float>(rgba(0)));
  shape_material_msg.mutable_ambient()->set_g(static_cast<float>(rgba(1)));
  shape_material_msg.mutable_ambient()->set_b(static_cast<float>(rgba(2)));
  shape_material_msg.mutable_ambient()->set_a(static_cast<float>(rgba(3)));

  shape_material_msg.mutable_diffuse()->set_r(static_cast<float>(rgba(0)));
  shape_material_msg.mutable_diffuse()->set_g(static_cast<float>(rgba(1)));
  shape_material_msg.mutable_diffuse()->set_b(static_cast<float>(rgba(2)));
  shape_material_msg.mutable_diffuse()->set_a(static_cast<float>(rgba(3)));

  shape_material_msg.mutable_specular()->set_r(static_cast<float>(rgba(0)));
  shape_material_msg.mutable_specular()->set_g(static_cast<float>(rgba(1)));
  shape_material_msg.mutable_specular()->set_b(static_cast<float>(rgba(2)));
  shape_material_msg.mutable_specular()->set_a(static_cast<float>(rgba(3)));

  return shape_material_msg;
}

bool toMsg(ignition::msgs::Scene& scene_msg,
           EntityManager& entity_manager,
           const tesseract_scene_graph::SceneGraph& scene_graph,
           const tesseract_common::TransformMap& link_transforms)
{
  scene_msg.set_name("scene");
  ignition::msgs::Model* model = scene_msg.add_model();
  model->set_name(scene_graph.getName());
  model->set_id(static_cast<unsigned>(entity_manager.addModel(scene_graph.getName())));
  for (const auto& link : scene_graph.getLinks())
  {
    ignition::msgs::Link* link_msg = model->add_link();
    link_msg->set_name(link->getName());
    link_msg->set_id(static_cast<unsigned>(entity_manager.addLink(link->getName())));
    link_msg->mutable_pose()->CopyFrom(
        ignition::msgs::Convert(ignition::math::eigen3::convert(link_transforms.at(link->getName()))));

    int cnt = 0;
    for (const auto& vs : link->visual)
    {
      std::string gv_name = link->getName() + std::to_string(++cnt);
      switch (vs->geometry->getType())
      {
        case tesseract_geometry::GeometryType::BOX:
        {
          ignition::msgs::Visual* gv_msg = link_msg->add_visual();
          gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
          gv_msg->set_name(gv_name);
          gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(vs->origin)));

          ignition::msgs::Geometry geometry_msg;
          geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_BOX);

          auto shape = std::static_pointer_cast<const tesseract_geometry::Box>(vs->geometry);
          ignition::msgs::BoxGeom shape_geometry_msg;
          shape_geometry_msg.mutable_size()->CopyFrom(
              ignition::msgs::Convert(ignition::math::Vector3d(shape->getX(), shape->getY(), shape->getZ())));
          geometry_msg.mutable_box()->CopyFrom(shape_geometry_msg);
          gv_msg->mutable_geometry()->CopyFrom(geometry_msg);

          if (vs->material != nullptr && vs->material->getName() != "default_tesseract_material" &&
              vs->material->texture_filename.empty())
          {
            gv_msg->mutable_material()->CopyFrom(convert(vs->material->color));
          }

          gv_msg->set_parent_name(link->getName());
          break;
        }
        case tesseract_geometry::GeometryType::SPHERE:
        {
          ignition::msgs::Visual* gv_msg = link_msg->add_visual();
          gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
          gv_msg->set_name(gv_name);
          gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(vs->origin)));

          ignition::msgs::Geometry geometry_msg;
          geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_SPHERE);

          auto shape = std::static_pointer_cast<const tesseract_geometry::Sphere>(vs->geometry);
          ignition::msgs::SphereGeom shape_geometry_msg;
          shape_geometry_msg.set_radius(shape->getRadius());
          geometry_msg.mutable_sphere()->CopyFrom(shape_geometry_msg);
          gv_msg->mutable_geometry()->CopyFrom(geometry_msg);

          if (vs->material != nullptr && vs->material->getName() != "default_tesseract_material" &&
              vs->material->texture_filename.empty())
          {
            gv_msg->mutable_material()->CopyFrom(convert(vs->material->color));
          }

          gv_msg->set_parent_name(link->getName());
          break;
        }
        case tesseract_geometry::GeometryType::CYLINDER:
        {
          ignition::msgs::Visual* gv_msg = link_msg->add_visual();
          gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
          gv_msg->set_name(gv_name);
          gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(vs->origin)));

          ignition::msgs::Geometry geometry_msg;
          geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_CYLINDER);

          auto shape = std::static_pointer_cast<const tesseract_geometry::Cylinder>(vs->geometry);
          ignition::msgs::CylinderGeom shape_geometry_msg;
          shape_geometry_msg.set_radius(shape->getRadius());
          shape_geometry_msg.set_length(shape->getLength());
          geometry_msg.mutable_cylinder()->CopyFrom(shape_geometry_msg);
          gv_msg->mutable_geometry()->CopyFrom(geometry_msg);

          if (vs->material != nullptr && vs->material->getName() != "default_tesseract_material" &&
              vs->material->texture_filename.empty())
          {
            gv_msg->mutable_material()->CopyFrom(convert(vs->material->color));
          }

          gv_msg->set_parent_name(link->getName());
          break;
        }
          //      case tesseract_geometry::GeometryType::CONE:
          //      {
          //          ignition::msgs::Visual* gv_msg = link_msg->add_visual();
          //          gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
          //          gv_msg->set_name(gv_name);
          //          gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(vs->origin)));

          //          ignition::msgs::Geometry geometry_msg;
          //          geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_CONE);

          //          auto shape = std::static_pointer_cast<const tesseract_geometry::Cone>(vs->geometry);
          //          ignition::msgs::ConeGeom shape_geometry_msg;
          //          shape_geometry_msg.set_radius(shape->getRadius());
          //          shape_geometry_msg.set_length(shape->getLength());
          //          geometry_msg.mutable_sphere()->CopyFrom(shape_geometry_msg);
          //          gv_msg->mutable_geometry()->CopyFrom(geometry_msg);
          //
          //          if (vs->material != nullptr && vs->material->getName() != "default_tesseract_material" &&
          //          vs->material->texture_filename.empty())
          //          {
          //            gv_msg->mutable_material()->CopyFrom(convert(vs->material->color));
          //          }
          //
          //          gv_msg->set_parent_name(link->getName());
          //          break;
          //      }

          //        case tesseract_geometry::GeometryType::CAPSULE:
          //        {
          //          ignition::msgs::Visual* gv_msg = link_msg->add_visual();
          //          gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
          //          gv_msg->set_name(gv_name);
          //          gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(vs->origin)));

          //          ignition::msgs::Geometry geometry_msg;
          //          geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_CAPSULE);

          //          auto shape = std::static_pointer_cast<const tesseract_geometry::Capsule>(vs->geometry);
          //          ignition::msgs::CapsuleGeom shape_geometry_msg;
          //          shape_geometry_msg.set_radius(shape->getRadius());
          //          shape_geometry_msg.set_length(shape->getLength());
          //          geometry_msg.mutable_sphere()->CopyFrom(shape_geometry_msg);
          //          gv_msg->mutable_geometry()->CopyFrom(geometry_msg);
          //
          //          if (vs->material != nullptr && vs->material->getName() != "default_tesseract_material" &&
          //          vs->material->texture_filename.empty())
          //          {
          //            gv_msg->mutable_material()->CopyFrom(convert(vs->material->color));
          //          }
          //
          //          gv_msg->set_parent_name(link->getName());
          //          break;
          //        }
        case tesseract_geometry::GeometryType::MESH:
        {
          ignition::msgs::Visual* gv_msg = link_msg->add_visual();
          gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
          gv_msg->set_name(gv_name);
          gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(vs->origin)));

          ignition::msgs::Geometry geometry_msg;
          geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_MESH);

          auto shape = std::static_pointer_cast<const tesseract_geometry::Mesh>(vs->geometry);
          auto resource = shape->getResource();
          if (resource)
          {
            ignition::msgs::MeshGeom shape_geometry_msg;
            shape_geometry_msg.set_filename(resource->getFilePath());
            shape_geometry_msg.mutable_scale()->CopyFrom(
                ignition::msgs::Convert(ignition::math::eigen3::convert(shape->getScale())));
            geometry_msg.mutable_mesh()->CopyFrom(shape_geometry_msg);
            gv_msg->mutable_geometry()->CopyFrom(geometry_msg);

            if (!isMeshWithColor(resource->getFilePath()) && vs->material != nullptr &&
                vs->material->getName() != "default_tesseract_material" && vs->material->texture_filename.empty())
            {
              gv_msg->mutable_material()->CopyFrom(convert(vs->material->color));
            }

            gv_msg->set_parent_name(link->getName());
          }
          else
          {
            assert(false);
          }

          break;
        }
        case tesseract_geometry::GeometryType::CONVEX_MESH:
        {
          ignition::msgs::Visual* gv_msg = link_msg->add_visual();
          gv_msg->set_id(static_cast<unsigned>(entity_manager.addVisual(gv_name)));
          gv_msg->set_name(gv_name);
          gv_msg->mutable_pose()->CopyFrom(ignition::msgs::Convert(ignition::math::eigen3::convert(vs->origin)));

          ignition::msgs::Geometry geometry_msg;
          geometry_msg.set_type(ignition::msgs::Geometry::Type::Geometry_Type_MESH);

          auto shape = std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(vs->geometry);
          auto resource = shape->getResource();
          if (resource)
          {
            ignition::msgs::MeshGeom shape_geometry_msg;
            shape_geometry_msg.set_filename(resource->getFilePath());
            shape_geometry_msg.mutable_scale()->CopyFrom(
                ignition::msgs::Convert(ignition::math::eigen3::convert(shape->getScale())));
            geometry_msg.mutable_mesh()->CopyFrom(shape_geometry_msg);
            gv_msg->mutable_geometry()->CopyFrom(geometry_msg);

            if (!isMeshWithColor(resource->getFilePath()) && vs->material != nullptr &&
                vs->material->getName() != "default_tesseract_material" && vs->material->texture_filename.empty())
            {
              gv_msg->mutable_material()->CopyFrom(convert(vs->material->color));
            }

            gv_msg->set_parent_name(link->getName());
          }
          else
          {
            assert(false);
          }

          break;
        }
          //        case tesseract_geometry::GeometryType::OCTREE:
          //        {
          //          auto shape = std::static_pointer_cast<const tesseract_geometry::Octree>(vs->geometry);

          //          // TODO: Need to implement
          //          assert(false);
          //          break;
          //        }
        default:
        {
          ignerr << "This geometric shape type " << static_cast<int>(vs->geometry->getType()) << " is not supported";
          break;
        }
      }
    }
  }
  return true;
}

}  // namespace tesseract_visualization
