#ifndef TESSERACT_RVIZ_STATE_MONITORING_H
#define TESSERACT_RVIZ_STATE_MONITORING_H

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/JointState.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP
#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_environment/core/environment.h>
#endif

namespace rviz
{
class Property;
class RosTopicProperty;
}

namespace tesseract_rviz
{

class JointStateMonitorWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<JointStateMonitorWidget>;
  using ConstPtr = std::shared_ptr<const JointStateMonitorWidget>;

  JointStateMonitorWidget(rviz::Property* widget, rviz::Display* display);

  virtual ~JointStateMonitorWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract_environment::EnvironmentPtr env,
                    rviz::DisplayContext* context,
                    ros::NodeHandle update_nh);

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

private Q_SLOTS:
  void changedJointStateTopic();

protected:
  rviz::Property* widget_;
  rviz::Display* display_;
  VisualizationWidget::Ptr visualization_;
  tesseract_environment::EnvironmentPtr env_;
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_subscriber_;
  rviz::RosTopicProperty* joint_state_topic_property_;
  bool update_required_;

  void newJointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
};
}  // namespace tesseract_rviz
#endif // TESSERACT_RVIZ_STATE_MONITORING_H
