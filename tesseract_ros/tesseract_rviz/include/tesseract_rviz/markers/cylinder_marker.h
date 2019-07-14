/**
 * @file cylinder_marker.h
 * @brief Cylinder marker
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
#ifndef TESSERACT_RVIZ_MARKERS_CYLINDER_MARKER_H
#define TESSERACT_RVIZ_MARKERS_CYLINDER_MARKER_H

#ifndef Q_MOC_RUN
#include <tesseract_rviz/markers/marker_base.h>
#include <OgreVector3.h>
#include <string>
#endif

namespace rviz
{
class Shape;
}

namespace tesseract_rviz
{
class CylinderMarker : public MarkerBase
{
public:
  using Ptr = boost::shared_ptr<CylinderMarker>;
  using ConstPtr = boost::shared_ptr<const CylinderMarker>;

  CylinderMarker(const std::string& ns,
                 const int id,
                 rviz::DisplayContext* context,
                 Ogre::SceneNode* parent_node,
                 float radius = 1,
                 float height = 2);
  ~CylinderMarker() override;

  void setScale(Ogre::Vector3 scale) override;
  Ogre::Vector3 getScale() const override;

  void setRadius(float radius);
  float getRadius() const;

  void setHeight(float height);
  float getHeight() const;

  void setColor(Ogre::ColourValue color) override;

  std::set<Ogre::MaterialPtr> getMaterials() override;

protected:
  rviz::Shape* shape_;
  Ogre::Vector3 scale_;
  float radius_;
  float height_;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_MARKERS_CYLINDER_MARKER_H
