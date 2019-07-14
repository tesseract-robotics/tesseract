/**
 * @file tesseract_manipulation_display.h
 * @brief Provides the ability to manipulate different manipulators in rviz.
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
#ifndef TESSERACT_RVIZ_TESSERACT_MANIPULATION_DISPLAY_H
#define TESSERACT_RVIZ_TESSERACT_MANIPULATION_DISPLAY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tesseract/tesseract.h>
#endif
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_rviz/render_tools/environment_widget.h>
#include <tesseract_rviz/render_tools/manipulation_widget.h>

namespace tesseract_rviz
{
class TesseractManipulationDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TesseractManipulationDisplay();

  ~TesseractManipulationDisplay() override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void setName(const QString& name) override;

protected:
  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

  // ROS Node Handle
  ros::NodeHandle nh_;

  tesseract::Tesseract::Ptr tesseract_;
  VisualizationWidget::Ptr visualization_;
  EnvironmentWidget::Ptr environment_monitor_;
  ManipulationWidget::Ptr manipulation_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_TESSERACT_MANIPULATION_DISPLAY_H
