#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_msgs/TesseractState.h>
#include <std_msgs/ColorRGBA.h>
#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#endif
#include <tesseract_rviz/render_tools/visualization_widget.h>

namespace rviz
{
class Property;
class RosTopicProperty;
class StringProperty;
class FloatProperty;
}  // namespace rviz

namespace tesseract_rviz
{
class EnvironmentWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<EnvironmentWidget>;
  using ConstPtr = std::shared_ptr<const EnvironmentWidget>;

  EnvironmentWidget(rviz::Property* widget, rviz::Display* display, const std::string& widget_ns = std::string());

  virtual ~EnvironmentWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract::Tesseract::Ptr tesseract,
                    rviz::DisplayContext* context,
                    const ros::NodeHandle& update_nh,
                    bool update_state,
                    const QString& tesseract_state_topic = "");

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

  /** @brief Returns the ID of this EnvironmentWidget instance which is associated with the default namespace */
  int getId() const { return environment_widget_id_; }

private Q_SLOTS:
  void changedURDFDescription();
  void changedEnvironmentNamespace();
  void changedRootLinkName();
  void changedTesseractStateTopic();
  void changedURDFSceneAlpha();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:
  rviz::Property* widget_;
  rviz::Display* display_;
  VisualizationWidget::Ptr visualization_;
  tesseract::Tesseract::Ptr tesseract_;
  ros::NodeHandle nh_;
  ros::Subscriber tesseract_state_subscriber_;        /**< @brief subscriber for getting environment updates */
  ros::ServiceServer modify_environment_server_;      /**< @brief host a service for modifying the environment */
  ros::ServiceServer get_environment_changes_server_; /**< @brief host a service for getting the environment changes */
  bool update_required_;
  bool update_state_;    /**< @brief Update visualization current state from environment message */
  bool load_tesseract_;  // for delayed initialization
  std::map<std::string, std_msgs::ColorRGBA> highlights_;

  void loadEnvironment();

  /** @brief Set the scene node's position, given the target frame and the planning frame */
  //  void calculateOffsetPosition();

  //  void setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors);
  //  void setLinkColor(Robot* robot, const std::string& link_name, const QColor& color);
  //  void unsetLinkColor(Robot* robot, const std::string& link_name);

  /** @brief Callback for new tesseract state message */
  void newTesseractStateCallback(const tesseract_msgs::TesseractStateConstPtr& state_msg);

  /** @brief Callback for modifying the environment via service request */
  bool modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                 tesseract_msgs::ModifyEnvironmentResponse& res);

  /** @brief Callback for get the environment changes via service request */
  bool getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                     tesseract_msgs::GetEnvironmentChangesResponse& res);

  /** @brief Apply a list of commands to the environment. This used by both services and topics for updating environment
   * visualization */
  bool applyEnvironmentCommands(const std::vector<tesseract_msgs::EnvironmentCommand>& commands);

  rviz::Property* main_property_;
  rviz::StringProperty* urdf_description_property_;
  rviz::StringProperty* environment_namespace_property_;
  rviz::RosTopicProperty* tesseract_state_topic_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::BoolProperty* enable_link_highlight_;
  rviz::BoolProperty* enable_visual_visible_;
  rviz::BoolProperty* enable_collision_visible_;
  rviz::BoolProperty* show_all_links_;

private:
  /** @brief Keeps track of how many EnvironmentWidgets have been created for the default namespace */
  static int environment_widget_counter_;
  /** @brief Keeps track of which EnvironmentWidget this is */
  int environment_widget_id_;

  std::string widget_ns_;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
