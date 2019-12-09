#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/window_manager_interface.h>

#include <tesseract_rosutils/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/utils.h>
#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_rviz/render_tools/link_widget.h>
#include <tesseract_rviz/render_tools/environment_widget.h>

namespace tesseract_rviz
{
const std::string DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string DEFAULT_MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";

EnvironmentWidget::EnvironmentWidget(rviz::Property* widget, rviz::Display* display, const std::string& widget_ns)
  : widget_(widget)
  , display_(display)
  , visualization_(nullptr)
  , tesseract_(nullptr)
  , update_required_(false)
  , update_state_(true)
  , load_tesseract_(true)
{
  environment_widget_counter_++;
  if (widget_ns.empty())
  {
    environment_widget_id_ = environment_widget_counter_;
    widget_ns_ = "env_widget_" + std::to_string(environment_widget_id_) + "/";
  }
  else
  {
    widget_ns_ = widget_ns;
  }

  main_property_ = new rviz::Property("Environment", "", "Tesseract Environment", widget_, nullptr, this);

  urdf_description_property_ = new rviz::StringProperty("URDF Description",
                                                        "robot_description",
                                                        "The name of the ROS parameter where the URDF for the robot is "
                                                        "loaded",
                                                        main_property_,
                                                        SLOT(changedURDFDescription()),
                                                        this);

  environment_namespace_property_ = new rviz::StringProperty("Interface Namespace",
                                                             QString::fromUtf8(widget_ns_.c_str()),
                                                             "The namespace used for the service interface associated "
                                                             "with the environment",
                                                             main_property_,
                                                             SLOT(changedEnvironmentNamespace()),
                                                             this);

  tesseract_state_topic_property_ =
      new rviz::RosTopicProperty("Tesseract State Topic",
                                 "display_tesseract_state",
                                 ros::message_traits::datatype<tesseract_msgs::TesseractState>(),
                                 "The topic on which the tesseract_msgs::TesseractState messages are received",
                                 main_property_,
                                 SLOT(changedTesseractStateTopic()),
                                 this);

  root_link_name_property_ = new rviz::StringProperty("Root Link",
                                                      "",
                                                      "Shows the name of the root link for the urdf",
                                                      main_property_,
                                                      SLOT(changedRootLinkName()),
                                                      this);
  root_link_name_property_->setReadOnly(true);

  alpha_property_ = new rviz::FloatProperty("Alpha",
                                            1.0f,
                                            "Specifies the alpha for the links with geometry",
                                            main_property_,
                                            SLOT(changedURDFSceneAlpha()),
                                            this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  enable_link_highlight_ = new rviz::BoolProperty("Show Highlights",
                                                  true,
                                                  "Specifies whether link highlighting is enabled",
                                                  main_property_,
                                                  SLOT(changedEnableLinkHighlight()),
                                                  this);
  enable_visual_visible_ = new rviz::BoolProperty("Show Visual",
                                                  true,
                                                  "Whether to display the visual representation of the environment.",
                                                  main_property_,
                                                  SLOT(changedEnableVisualVisible()),
                                                  this);

  enable_collision_visible_ = new rviz::BoolProperty("Show Collision",
                                                     false,
                                                     "Whether to display the collision "
                                                     "representation of the environment.",
                                                     main_property_,
                                                     SLOT(changedEnableCollisionVisible()),
                                                     this);

  show_all_links_ = new rviz::BoolProperty(
      "Show All Links", true, "Toggle all links visibility on or off.", main_property_, SLOT(changedAllLinks()), this);
}

EnvironmentWidget::~EnvironmentWidget() = default;

void EnvironmentWidget::onInitialize(VisualizationWidget::Ptr visualization,
                                     tesseract::Tesseract::Ptr tesseract,
                                     rviz::DisplayContext* /*context*/,
                                     const ros::NodeHandle& update_nh,
                                     bool update_state,
                                     const QString& tesseract_state_topic)
{
  visualization_ = std::move(visualization);
  tesseract_ = std::move(tesseract);
  nh_ = update_nh;
  update_state_ = update_state;

  tesseract_state_topic_property_->setString(tesseract_state_topic);

  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  changedTesseractStateTopic();
}

void EnvironmentWidget::onEnable()
{
  load_tesseract_ = true;  // allow loading of robot model in update()
  //  calculateOffsetPosition();
}

void EnvironmentWidget::onDisable()
{
  tesseract_state_subscriber_.shutdown();
  if (visualization_)
    visualization_->setVisible(false);
}

void EnvironmentWidget::onUpdate()
{
  if (load_tesseract_)
  {
    loadEnvironment();
    update_required_ = true;
  }

  //  calculateOffsetPosition();

  if (visualization_ && update_required_ && tesseract_->getEnvironment())
  {
    update_required_ = false;
    visualization_->update(tesseract_->getEnvironment()->getCurrentState()->transforms);
  }
}

void EnvironmentWidget::onReset() { load_tesseract_ = true; }

void EnvironmentWidget::changedAllLinks()
{
  rviz::Property* links_prop = widget_->subProp("Links");
  QVariant value(show_all_links_->getBool());

  for (int i = 0; i < links_prop->numChildren(); ++i)
  {
    rviz::Property* link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

// void TesseractStateDisplay::setHighlightedLink(const std::string& link_name, const std_msgs::ColorRGBA& color)
//{
//  RobotLink* link = state_->getRobot().getLink(link_name);
//  if (link)
//  {
//    link->setColor(color.r, color.g, color.b);
//    link->setRobotAlpha(color.a * alpha_property_->getFloat());
//  }
//}

// void TesseractStateDisplay::unsetHighlightedLink(const std::string& link_name)
//{
//  RobotLink* link = state_->getRobot().getLink(link_name);
//  if (link)
//  {
//    link->unsetColor();
//    link->setRobotAlpha(alpha_property_->getFloat());
//  }
//}

void EnvironmentWidget::changedEnableLinkHighlight()
{
  //  if (enable_link_highlight_->getBool())
  //  {
  //    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end();
  //    ++it)
  //    {
  //      setHighlightedLink(it->first, it->second);
  //    }
  //  }
  //  else
  //  {
  //    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end();
  //    ++it)
  //    {
  //      unsetHighlightedLink(it->first);
  //    }
  //  }
}

void EnvironmentWidget::changedEnableVisualVisible()
{
  visualization_->setVisualVisible(enable_visual_visible_->getBool());
}

void EnvironmentWidget::changedEnableCollisionVisible()
{
  visualization_->setCollisionVisible(enable_collision_visible_->getBool());
}

// static bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
//{
//  return a.r != b.r || a.g != b.g || a.b != b.b || a.a != b.a;
//}

void EnvironmentWidget::changedURDFDescription()
{
  if (display_->isEnabled())
    onReset();
}

void EnvironmentWidget::changedEnvironmentNamespace()
{
  widget_ns_ = environment_namespace_property_->getStdString();

  if (load_tesseract_)
    return;

  // Need to kill services and relaunch with new name
  modify_environment_server_.shutdown();
  get_environment_changes_server_.shutdown();

  modify_environment_server_ = nh_.advertiseService(
      widget_ns_ + DEFAULT_MODIFY_ENVIRONMENT_SERVICE, &EnvironmentWidget::modifyEnvironmentCallback, this);

  get_environment_changes_server_ = nh_.advertiseService(
      widget_ns_ + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE, &EnvironmentWidget::getEnvironmentChangesCallback, this);
}

void EnvironmentWidget::changedRootLinkName() {}
void EnvironmentWidget::changedURDFSceneAlpha()
{
  if (visualization_)
  {
    visualization_->setAlpha(alpha_property_->getFloat());
    update_required_ = true;
  }
}

bool EnvironmentWidget::applyEnvironmentCommands(const std::vector<tesseract_msgs::EnvironmentCommand>& commands)
{
  update_required_ = true;
  for (const auto& command : commands)
  {
    switch (command.command)
    {
      case tesseract_msgs::EnvironmentCommand::ADD:
      {
        tesseract_scene_graph::Link link = tesseract_rosutils::fromMsg(command.add_link);
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.add_joint);

        if (!visualization_->addLink(link) || !visualization_->addJoint(joint))
          return false;

        if (!tesseract_->getEnvironment()->addLink(link, joint))
          return false;

        break;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_LINK:
      {
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.move_link_joint);

        std::vector<tesseract_scene_graph::Joint::ConstPtr> joints =
            tesseract_->getEnvironment()->getSceneGraph()->getInboundJoints(joint.child_link_name);
        assert(joints.size() == 1);

        if (!visualization_->removeJoint(joints[0]->getName()) || !visualization_->addJoint(joint))
          return false;

        if (!tesseract_->getEnvironment()->moveLink(joint))
          return false;

        break;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_JOINT:
      {
        if (!visualization_->moveJoint(command.move_joint_name, command.move_joint_parent_link))
          return false;

        if (!tesseract_->getEnvironment()->moveJoint(command.move_joint_name, command.move_joint_parent_link))
          return false;

        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_LINK:
      {
        if (tesseract_->getEnvironment()->getLink(command.remove_link) == nullptr)
        {
          ROS_WARN("Tried to remove link (%s) that does not exist", command.remove_link.c_str());
          return false;
        }

        std::vector<tesseract_scene_graph::Joint::ConstPtr> joints =
            tesseract_->getEnvironment()->getSceneGraph()->getInboundJoints(command.remove_link);
        assert(joints.size() <= 1);

        // get child link names to remove
        std::vector<std::string> child_link_names =
            tesseract_->getEnvironment()->getSceneGraph()->getLinkChildrenNames(command.remove_link);

        if (!visualization_->removeJoint(joints[0]->getName()))
          return false;

        if (!visualization_->removeLink(command.remove_link))
          return false;

        for (const auto& link_name : child_link_names)
        {
          if (!visualization_->removeLink(link_name))
            return false;

          std::vector<tesseract_scene_graph::Joint::ConstPtr> joints =
              tesseract_->getEnvironment()->getSceneGraph()->getInboundJoints(link_name);
          if (joints.size() == 1)
          {
            if (!visualization_->removeJoint(joints[0]->getName()))
              return false;
          }
        }

        if (!tesseract_->getEnvironment()->removeLink(command.remove_link))
          return false;

        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_JOINT:
      {
        tesseract_scene_graph::Joint::ConstPtr remove_joint =
            tesseract_->getEnvironment()->getJoint(command.remove_joint);
        std::vector<tesseract_scene_graph::Joint::ConstPtr> joints =
            tesseract_->getEnvironment()->getSceneGraph()->getInboundJoints(remove_joint->child_link_name);
        assert(joints.size() <= 1);

        // get child link names to remove
        std::vector<std::string> child_link_names =
            tesseract_->getEnvironment()->getSceneGraph()->getLinkChildrenNames(remove_joint->child_link_name);

        if (!visualization_->removeJoint(joints[0]->getName()))
          return false;

        if (!visualization_->removeLink(remove_joint->child_link_name))
          return false;

        for (const auto& link_name : child_link_names)
        {
          if (!visualization_->removeLink(link_name))
            return false;

          std::vector<tesseract_scene_graph::Joint::ConstPtr> joints =
              tesseract_->getEnvironment()->getSceneGraph()->getInboundJoints(link_name);
          if (joints.size() == 1)
          {
            if (!visualization_->removeJoint(joints[0]->getName()))
              return false;
          }
        }

        if (!tesseract_->getEnvironment()->removeJoint(command.remove_joint))
          return false;

        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_ORIGIN:
      {
        assert(false);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN:
      {
        Eigen::Isometry3d pose;
        tf::poseMsgToEigen(command.change_joint_origin_pose, pose);
        tesseract_->getEnvironment()->changeJointOrigin(command.change_joint_origin_name, pose);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED:
      {
        visualization_->setLinkCollisionEnabled(command.change_link_collision_enabled_name,
                                                command.change_link_collision_enabled_value);

        tesseract_->getEnvironment()->setLinkCollisionEnabled(command.change_link_collision_enabled_name,
                                                              command.change_link_collision_enabled_value);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY:
      {
        // TODO:: Need to update visualization.
        tesseract_->getEnvironment()->setLinkVisibility(command.change_link_visibility_name,
                                                        command.change_link_visibility_value);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION:
      {
        visualization_->addAllowedCollision(command.add_allowed_collision.link_1,
                                            command.add_allowed_collision.link_2,
                                            command.add_allowed_collision.reason);

        tesseract_->getEnvironment()->addAllowedCollision(command.add_allowed_collision.link_1,
                                                          command.add_allowed_collision.link_2,
                                                          command.add_allowed_collision.reason);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION:
      {
        visualization_->removeAllowedCollision(command.add_allowed_collision.link_1,
                                               command.add_allowed_collision.link_2);

        tesseract_->getEnvironment()->removeAllowedCollision(command.add_allowed_collision.link_1,
                                                             command.add_allowed_collision.link_2);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK:
      {
        visualization_->removeAllowedCollision(command.remove_allowed_collision_link);

        tesseract_->getEnvironment()->removeAllowedCollision(command.remove_allowed_collision_link);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE:
      {
        if (!tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), command.joint_state))
          return false;

        break;
      }
    }
  }
  return true;
}

bool EnvironmentWidget::modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                                  tesseract_msgs::ModifyEnvironmentResponse& res)
{
  if (!tesseract_->getEnvironment() || req.id != tesseract_->getEnvironment()->getName() ||
      static_cast<int>(req.revision) != tesseract_->getEnvironment()->getRevision())
    return false;

  res.success = applyEnvironmentCommands(req.commands);
  res.revision = static_cast<std::size_t>(tesseract_->getEnvironment()->getRevision());
  return res.success;
}

bool EnvironmentWidget::getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                                      tesseract_msgs::GetEnvironmentChangesResponse& res)
{
  if (static_cast<int>(req.revision) > tesseract_->getEnvironment()->getRevision())
  {
    ROS_WARN("Requested changes for id %d greater than current change id %d",
             static_cast<int>(req.revision),
             tesseract_->getEnvironment()->getRevision());
    res.success = false;
    return false;
  }

  res.id = tesseract_->getEnvironment()->getName();
  res.revision = static_cast<std::size_t>(tesseract_->getEnvironment()->getRevision());
  const tesseract_environment::Commands& commands = tesseract_->getEnvironment()->getCommandHistory();
  for (std::size_t i = (req.revision == 0 ? 0 : req.revision - 1); i < commands.size(); ++i)
  {
    tesseract_msgs::EnvironmentCommand command_msg;
    if (!tesseract_rosutils::toMsg(command_msg, *(commands[i])))
    {
      res.success = false;
      return false;
    }

    res.commands.push_back(command_msg);
  }

  res.success = true;
  return res.success;
}

void EnvironmentWidget::changedTesseractStateTopic()
{
  tesseract_state_subscriber_.shutdown();

  // reset model to default state, we don't want to show previous messages
  //  current_state_ = nullptr;
  update_required_ = true;

  tesseract_state_subscriber_ = nh_.subscribe(
      tesseract_state_topic_property_->getStdString(), 10, &EnvironmentWidget::newTesseractStateCallback, this);
}

void EnvironmentWidget::newTesseractStateCallback(const tesseract_msgs::TesseractStateConstPtr& state_msg)
{
  if (!tesseract_->getEnvironment())
    return;

  int current_version = tesseract_->getEnvironment()->getRevision();
  if (!tesseract_->getEnvironment() || state_msg->id != tesseract_->getEnvironment()->getName() ||
      current_version > static_cast<int>(state_msg->revision))
    return;

  if (update_state_)
    if (tesseract_rosutils::processMsg(tesseract_->getEnvironment(), state_msg->joint_state))
      update_required_ = true;

  if (static_cast<int>(state_msg->revision) > current_version)
  {
    std::vector<tesseract_msgs::EnvironmentCommand> commands;
    for (std::size_t i = (current_version == 0 ? 0 : static_cast<std::size_t>(current_version) - 1ul);
         i < state_msg->revision;
         ++i)
      commands.push_back(state_msg->commands[i]);

    if (!commands.empty())
      applyEnvironmentCommands(commands);
  }

  //  setLinkColor(state_msg->object_colors);
  //  setHighlightedLinks(state_msg->highlight_links);
  //  update_state_ = true;
}

// void TesseractStateDisplay::setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors)
//{
//  assert(false);
//  for (tesseract_msgs::TesseractState::_object_colors_type::const_iterator it = link_colors.begin();
//       it != link_colors.end();
//       ++it)
//  {
//    setLinkColor(it->name,
//                 QColor(static_cast<int>(it->visual[0].r * 255),
//                        static_cast<int>(it->visual[0].g * 255),
//                        static_cast<int>(it->visual[0].b * 255)));
//  }
//}

// void TesseractStateDisplay::setLinkColor(const std::string& link_name, const QColor& color)
//{
//  setLinkColor(&state_->getRobot(), link_name, color);
//}

// void TesseractStateDisplay::unsetLinkColor(const std::string& link_name)
//{
//  unsetLinkColor(&state_->getRobot(), link_name);
//}

// void TesseractStateDisplay::setLinkColor(Robot* robot, const std::string& link_name, const QColor& color)
//{
//  RobotLink* link = robot->getLink(link_name);

//  // Check if link exists
//  if (link)
//    link->setColor(
//        static_cast<float>(color.redF()), static_cast<float>(color.greenF()), static_cast<float>(color.blueF()));
//}

// void TesseractStateDisplay::unsetLinkColor(Robot* robot, const std::string& link_name)
//{
//  RobotLink* link = robot->getLink(link_name);

//  // Check if link exists
//  if (link)
//    link->unsetColor();
//}

void EnvironmentWidget::loadEnvironment()
{
  load_tesseract_ = false;
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(urdf_description_property_->getString().toStdString(), urdf_xml_string);
  nh_.getParam(urdf_description_property_->getString().toStdString() + "_semantic", srdf_xml_string);

  // Load URDF model
  if (urdf_xml_string.empty())
  {
    load_tesseract_ = true;
    // TODO:
    //    setStatus(rviz::StatusProperty::Error, "TesseractState", "No URDF model loaded");
  }
  else
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    if (tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    {
      visualization_->clear();
      visualization_->load(tesseract_->getEnvironment()->getSceneGraph(), true, true, true, true);
      bool oldState = root_link_name_property_->blockSignals(true);
      root_link_name_property_->setStdString(tesseract_->getEnvironment()->getRootLinkName());
      root_link_name_property_->blockSignals(oldState);
      update_required_ = true;
      //        setStatus(rviz::StatusProperty::Ok, "Tesseract", "Tesseract Environment Loaded Successfully");

      changedEnableVisualVisible();
      changedEnableCollisionVisible();
      visualization_->setVisible(true);

      if (modify_environment_server_.getService().empty())
      {
        modify_environment_server_ = nh_.advertiseService(
            widget_ns_ + DEFAULT_MODIFY_ENVIRONMENT_SERVICE, &EnvironmentWidget::modifyEnvironmentCallback, this);
      }

      if (get_environment_changes_server_.getService().empty())
      {
        get_environment_changes_server_ = nh_.advertiseService(widget_ns_ + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE,
                                                               &EnvironmentWidget::getEnvironmentChangesCallback,
                                                               this);
      }
    }
    else
    {
      //      setStatus(rviz::StatusProperty::Error, "Tesseract", "URDF file failed to parse");
    }
  }

  highlights_.clear();
}

int EnvironmentWidget::environment_widget_counter_ = -1;

}  // namespace tesseract_rviz
