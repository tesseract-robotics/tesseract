#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <rviz/display_context.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>

#include <tesseract_rosutils/utils.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_rviz/render_tools/joint_state_monitor_widget.h>

namespace tesseract_rviz
{
JointStateMonitorWidget::JointStateMonitorWidget(rviz::Property* widget, rviz::Display* display)
  : widget_(widget)
  , display_(display)
  , update_required_(false)
  , env_(nullptr)
  , visualization_(nullptr)
{
  joint_state_topic_property_ =
      new rviz::RosTopicProperty("Joint State Topic",
                                 "joint_states",
                                 ros::message_traits::datatype<sensor_msgs::JointState>(),
                                 "The topic on which the sensor_msgs::JointState messages are received",
                                 widget_,
                                 SLOT(changedJointStateTopic()),
                                 this);

 }

JointStateMonitorWidget::~JointStateMonitorWidget()
{
  joint_state_subscriber_.shutdown();
}

void JointStateMonitorWidget::onInitialize(VisualizationWidget::Ptr visualization,
                                tesseract_environment::EnvironmentPtr env,
                                rviz::DisplayContext* context,
                                ros::NodeHandle update_nh)
{
  visualization_ = std::move(visualization);
  env_ = std::move(env);
  nh_ = update_nh;
}

void JointStateMonitorWidget::changedJointStateTopic()
{
  joint_state_subscriber_.shutdown();

  joint_state_subscriber_ = nh_.subscribe(
      joint_state_topic_property_->getStdString(), 10, &JointStateMonitorWidget::newJointStateCallback, this);
}

void JointStateMonitorWidget::newJointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  if (!env_)
    return;

  tesseract_rosutils::processMsg(env_, *joint_state_msg);
  update_required_ = true;
}

void JointStateMonitorWidget::onEnable()
{
  changedJointStateTopic();
}

void JointStateMonitorWidget::onDisable()
{
  joint_state_subscriber_.shutdown();
}

void JointStateMonitorWidget::onUpdate()
{
  if (visualization_ && update_required_ && env_)
  {
    update_required_ = false;
    visualization_->update(env_->getCurrentState()->transforms);
  }
}

void JointStateMonitorWidget::onReset()
{
  changedJointStateTopic();
}

}  // namespace tesseract_rviz
