/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef TESSERACT_VISUALIZATION_MARKERS_MARKER_H
#define TESSERACT_VISUALIZATION_MARKERS_MARKER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <chrono>
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_visualization
{
enum class MarkerType : int
{
  /** @brief No type */
  NONE = 0,

  /** @brief Tesseract geometry */
  GEOMETRY = 1,

  /** @brief Axis primitive */
  AXIS = 2,

  /** @brief Arrow primitive */
  ARROW = 3,

  /** @brief Line strip primitive */
  LINE_STRIP = 4,

  /** @brief Line list primitive */
  LINE_LIST = 5,

  /** @brief Points primitive */
  POINTS = 6,

  /** @brief Text geometry */
  TEXT = 7,

  /** @brief Triangle fan primitive */
  TRIANGLE_FAN = 8,

  /** @brief Triangle list primitive */
  TRIANGLE_LIST = 9,

  /** @brief Triangle strip primitive */
  TRIANGLE_STRIP = 10,

  /** @brief Toolpath marker */
  TOOLPATH = 11,

  /** @brief Contact results marker */
  CONTACT_RESULTS = 12,

  // User defined types must be larger than this
  USER_DEFINED = 1000
};

class Marker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Marker>;
  using ConstPtr = std::shared_ptr<const Marker>;

  Marker() = default;
  virtual ~Marker() = default;
  Marker(const Marker&) = default;
  Marker& operator=(const Marker&) = default;
  Marker(Marker&&) = default;
  Marker& operator=(Marker&&) = default;

  /**
   * @brief Get the marker type
   * @return The type of the marker
   */
  virtual int getType() const = 0;

  /**
   * @brief The parent link name the marker should be attached to
   * @param parent_link The parent link name
   */
  virtual void setParentLink(std::string parent_link);

  /**
   * @brief Get the parent link name that marker should be attached to
   * @details If empty then it should be relative world coordinates
   */
  virtual const std::string& getParentLink() const;

  /**
   * @brief Set the lifetime of the this marker
   * @param lifetime The time at which the marker will be removed
   */
  virtual void setLifetime(const std::chrono::steady_clock::duration& lifetime);

  /**
   * @brief Get the lifetime of this Marker
   * @return The time at which the marker will be removed
   */
  virtual std::chrono::steady_clock::duration getLifetime() const;

  /**
   * @brief Set the layer of this Marker
   * @param layer Layer at which the marker will reside
   */
  virtual void setLayer(int layer);

  /**
   * @brief Get the layer of this Marker
   * @return The layer of the marker
   */
  virtual int getLayer() const;

  /**
   * @brief Set the marker scale
   * @param scale The marker scale
   */
  virtual void setScale(const Eigen::Vector3d& scale);

  /**
   * @brief Get the marker scale
   * @return The scale of the marker
   */
  virtual const Eigen::Vector3d& getScale() const;

protected:
  /** @brief The lifetime of this Marker */
  std::chrono::steady_clock::duration lifetime_{ 0 };

  /** @brief The parent link the marker is attched to. If empty relative to world */
  std::string parent_link_;

  /** @brief The layer of the marker */
  int layer_{ 0 };

  /** @brief The marker scale */
  Eigen::Vector3d scale_{ Eigen::Vector3d(1, 1, 1) };
};

}  // namespace tesseract_visualization
#endif  // TESSERACT_VISUALIZATION_MARKERS_MARKER_H
