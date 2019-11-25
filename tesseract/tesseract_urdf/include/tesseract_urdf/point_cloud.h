/**
 * @file point_cloud.h
 * @brief Parse PCL point cloud to octree from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_URDF_POINT_CLOUD_H
#define TESSERACT_URDF_POINT_CLOUD_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <pcl/io/pcd_io.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/utils.h>

namespace tesseract_urdf
{
class PointCloudStatusCategory : public tesseract_common::StatusCategory
{
public:
  PointCloudStatusCategory() : name_("PointCloudStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed point_cloud element!";
      case ErrorAttributeFileName:
        return "Missing or failed parsing point_cloud attribute 'filename'!";
      case ErrorAttributeResolution:
        return "Missing or failed parsing point_cloud attribute 'resolution'!";
      case ErrorImportingPointCloud:
        return "Error importing point_cloud from 'filename'!";
      case ErrorPointCloudEmpty:
        return "Error point cloud is empty!";
      case ErrorCreatingGeometry:
        return "Error create octree geometry type from point cloud!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorAttributeFileName = -1,
    ErrorAttributeResolution = -2,
    ErrorImportingPointCloud = -3,
    ErrorPointCloudEmpty = -4,
    ErrorCreatingGeometry = -5
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parsePointCloud(tesseract_geometry::Octree::Ptr& octree,
                                                         const tinyxml2::XMLElement* xml_element,
                                                         const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                                         tesseract_geometry::Octree::SubType shape_type,
                                                         const bool prune,
                                                         const int /*version*/)
{
  octree = nullptr;
  auto status_cat = std::make_shared<PointCloudStatusCategory>();

  std::string filename;
  if (QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(PointCloudStatusCategory::ErrorAttributeFileName, status_cat);

  double resolution;
  if (xml_element->QueryDoubleAttribute("resolution", &resolution) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(PointCloudStatusCategory::ErrorAttributeResolution,
                                                          status_cat);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tesseract_common::Resource::Ptr located_resource = locator->locateResource(filename);
  if (!located_resource->isFile())
  {
    // TODO: Handle point clouds that are not files
    CONSOLE_BRIDGE_logError("Point clouds can only be loaded from file");
    return std::make_shared<tesseract_common::StatusCode>(PointCloudStatusCategory::ErrorImportingPointCloud,
                                                          status_cat);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(located_resource->getFilePath(), *cloud) == -1)
    return std::make_shared<tesseract_common::StatusCode>(PointCloudStatusCategory::ErrorImportingPointCloud,
                                                          status_cat);

  if (cloud->points.empty())
    return std::make_shared<tesseract_common::StatusCode>(PointCloudStatusCategory::ErrorPointCloudEmpty, status_cat);

  auto geom = std::make_shared<tesseract_geometry::Octree>(*cloud, resolution, shape_type, prune);
  if (geom == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(PointCloudStatusCategory::ErrorCreatingGeometry, status_cat);

  octree = std::move(geom);
  return std::make_shared<tesseract_common::StatusCode>(PointCloudStatusCategory::Success, status_cat);
  ;
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_POINT_CLOUD_H
