#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/service_server.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <std_msgs/ColorRGBA.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP
#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_environment/core/environment.h>
#endif

namespace rviz
{
class Property;
class RosTopicProperty;
class StringProperty;
class FloatProperty;
}

namespace tesseract_rviz
{

class EnvironmentWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<EnvironmentWidget>;
  using ConstPtr = std::shared_ptr<const EnvironmentWidget>;

  EnvironmentWidget(rviz::Property* widget, rviz::Display* display);

  virtual ~EnvironmentWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract_environment::EnvironmentPtr env,
                    rviz::DisplayContext* context,
                    ros::NodeHandle update_nh);

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

private Q_SLOTS:
  void changedURDFDescription();
  void changedRootLinkName();
  void changedURDFSceneAlpha();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:


  rviz::Property* widget_;
  rviz::Display* display_;
  VisualizationWidget::Ptr visualization_;
  tesseract_environment::EnvironmentPtr env_;
  ros::NodeHandle nh_;
  ros::ServiceServer modify_environment_server_; /**< @brief host a service for modifying the environment */
  ros::ServiceServer get_environment_changes_server_; /**< @brief host a service for getting the environment changes */
  bool update_required_;
  bool load_env_;  // for delayed initialization
  std::map<std::string, std_msgs::ColorRGBA> highlights_;

  void loadEnvironment();

  /** @brief Set the scene node's position, given the target frame and the planning frame */
//  void calculateOffsetPosition();

//  void setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors);
//  void setLinkColor(Robot* robot, const std::string& link_name, const QColor& color);
//  void unsetLinkColor(Robot* robot, const std::string& link_name);

  /** @brief Callback for modifying the environment via service request */
  bool modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                 tesseract_msgs::ModifyEnvironmentResponse& res);

  /** @brief Callback for get the environment changes via service request */
  bool getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                     tesseract_msgs::GetEnvironmentChangesResponse& res);

  rviz::StringProperty* urdf_description_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::BoolProperty* enable_link_highlight_;
  rviz::BoolProperty* enable_visual_visible_;
  rviz::BoolProperty* enable_collision_visible_;
  rviz::BoolProperty* show_all_links_;
};
}  // namespace tesseract_rviz
#endif // TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
