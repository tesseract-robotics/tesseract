/**
 * @file tesseract_manipulation_display.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/package.h>
#include <tesseract_rosutils/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/render_tools/link_widget.h>
#include <tesseract_rviz/tesseract_manipulation_plugin/tesseract_manipulation_display.h>

namespace tesseract_rviz
{
TesseractManipulationDisplay::TesseractManipulationDisplay()
{
  tesseract_ = std::make_shared<tesseract::Tesseract>();
  environment_monitor_ = std::make_shared<EnvironmentWidget>(this, this);
  manipulation_ = std::make_shared<ManipulationWidget>(this, this);
}

TesseractManipulationDisplay::~TesseractManipulationDisplay() = default;
void TesseractManipulationDisplay::onInitialize()
{
  Display::onInitialize();

  visualization_ = std::make_shared<VisualizationWidget>(scene_node_, context_, "Tesseract State", this);
  visualization_->setCurrentStateVisible(false);
  visualization_->setStartStateVisible(true);

  environment_monitor_->onInitialize(visualization_, tesseract_, context_, nh_, true);
  manipulation_->onInitialize(scene_node_,
                              context_,
                              visualization_,
                              tesseract_,
                              nh_,
                              ManipulationWidget::ManipulatorState::START,
                              "/tesseract/manipulation_start_state");

  visualization_->setVisible(false);
}

void TesseractManipulationDisplay::reset()
{
  environment_monitor_->onReset();
  manipulation_->onReset();
  visualization_->clear();

  Display::reset();
}

void TesseractManipulationDisplay::onEnable()
{
  Display::onEnable();

  environment_monitor_->onEnable();
  manipulation_->onEnable();
}

void TesseractManipulationDisplay::onDisable()
{
  environment_monitor_->onDisable();
  manipulation_->onDisable();
  Display::onDisable();
}

void TesseractManipulationDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  environment_monitor_->onUpdate();
  manipulation_->onUpdate(wall_dt);
}

void TesseractManipulationDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  manipulation_->onNameChange(name);
}

}  // namespace tesseract_rviz
