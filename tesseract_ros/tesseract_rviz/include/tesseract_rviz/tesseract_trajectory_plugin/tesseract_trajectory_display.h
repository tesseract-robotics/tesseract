/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Wraps a trajectory_visualization playback class for Rviz into a stand alone display
*/

#ifndef TESSERACT_RVIZ_TESSERACT_TRAJECTORY_DISPLAY_PLUGIN
#define TESSERACT_RVIZ_TESSERACT_TRAJECTORY_DISPLAY_PLUGIN

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tesseract/tesseract.h>
#endif
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_rviz/render_tools/joint_state_monitor_widget.h>
#include <tesseract_rviz/render_tools/environment_widget.h>
#include <tesseract_rviz/render_tools/trajectory_monitor_widget.h>

namespace tesseract_rviz
{
class TesseractTrajectoryDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TesseractTrajectoryDisplay();

  ~TesseractTrajectoryDisplay() override;

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
  JointStateMonitorWidget::Ptr state_monitor_;
  EnvironmentWidget::Ptr environment_monitor_;
  TrajectoryMonitorWidget::Ptr trajectory_monitor_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_TESSERACT_TRAJECTORY_DISPLAY_PLUGIN
