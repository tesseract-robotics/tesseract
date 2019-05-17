#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
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
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_rviz/render_tools/link_widget.h>
#include <tesseract_rviz/render_tools/environment_widget.h>

namespace tesseract_rviz
{
const std::string DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string DEFAULT_MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";

EnvironmentWidget::EnvironmentWidget(rviz::Property* widget, rviz::Display* display)
  : widget_(widget)
  , display_(display)
  , update_required_(false)
  , env_(nullptr)
  , visualization_(nullptr)
{
  urdf_description_property_ =
      new rviz::StringProperty("URDF Description",
                               "robot_description",
                               "The name of the ROS parameter where the URDF for the robot is loaded",
                               widget_,
                               SLOT(changedURDFDescription()),
                               this);

  root_link_name_property_ = new rviz::StringProperty(
      "Root Link", "", "Shows the name of the root link for the urdf", widget_, SLOT(changedRootLinkName()), this);
  root_link_name_property_->setReadOnly(true);

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0f, "Specifies the alpha for the links with geometry", widget_, SLOT(changedURDFSceneAlpha()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  enable_link_highlight_ = new rviz::BoolProperty("Show Highlights",
                                                  true,
                                                  "Specifies whether link highlighting is enabled",
                                                  widget_,
                                                  SLOT(changedEnableLinkHighlight()),
                                                  this);
  enable_visual_visible_ = new rviz::BoolProperty("Show Visual",
                                                  true,
                                                  "Whether to display the visual representation of the environment.",
                                                  widget_,
                                                  SLOT(changedEnableVisualVisible()),
                                                  this);

  enable_collision_visible_ = new rviz::BoolProperty("Show Collision",
                                                     false,
                                                     "Whether to display the collision "
                                                     "representation of the environment.",
                                                     widget_,
                                                     SLOT(changedEnableCollisionVisible()),
                                                     this);

  show_all_links_ = new rviz::BoolProperty(
      "Show All Links", true, "Toggle all links visibility on or off.", widget_, SLOT(changedAllLinks()), this);


 }

EnvironmentWidget::~EnvironmentWidget()
{
}

void EnvironmentWidget::onInitialize(VisualizationWidget::Ptr visualization,
                                     tesseract_environment::EnvironmentPtr env,
                                     rviz::DisplayContext* context,
                                     ros::NodeHandle update_nh)
{
  visualization_ = std::move(visualization);
  env_ = std::move(env);
  nh_ = update_nh;

  modify_environment_server_ =
      nh_.advertiseService(DEFAULT_MODIFY_ENVIRONMENT_SERVICE, &EnvironmentWidget::modifyEnvironmentCallback, this);

  get_environment_changes_server_ =
      nh_.advertiseService(DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE, &EnvironmentWidget::getEnvironmentChangesCallback, this);

  changedEnableVisualVisible();
  changedEnableCollisionVisible();
}

void EnvironmentWidget::onEnable()
{
  load_env_ = true;  // allow loading of robot model in update()
//  calculateOffsetPosition();
}

void EnvironmentWidget::onDisable()
{
  if (visualization_)
    visualization_->setVisible(false);
}

void EnvironmentWidget::onUpdate()
{
  if (load_env_)
  {
    loadEnvironment();
    update_required_ = true;
  }

//  calculateOffsetPosition();

  if (visualization_ && update_required_ && env_)
  {
    update_required_ = false;
    visualization_->update(env_->getCurrentState()->transforms);
  }
}

void EnvironmentWidget::onReset()
{
  loadEnvironment();
}

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

//void TesseractStateDisplay::setHighlightedLink(const std::string& link_name, const std_msgs::ColorRGBA& color)
//{
//  RobotLink* link = state_->getRobot().getLink(link_name);
//  if (link)
//  {
//    link->setColor(color.r, color.g, color.b);
//    link->setRobotAlpha(color.a * alpha_property_->getFloat());
//  }
//}

//void TesseractStateDisplay::unsetHighlightedLink(const std::string& link_name)
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
//    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
//    {
//      setHighlightedLink(it->first, it->second);
//    }
//  }
//  else
//  {
//    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
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

static bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r != b.r || a.g != b.g || a.b != b.b || a.a != b.a;
}

void EnvironmentWidget::changedURDFDescription()
{
//  if (isEnabled())
//    reset();
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

bool EnvironmentWidget::modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest &req,
                                                   tesseract_msgs::ModifyEnvironmentResponse &res)
{
  if (!env_ || req.id != env_->getName() || req.revision != env_->getRevision())
  {
    return false;
  }

  update_required_ = true;
  for (const auto& command : req.commands)
  {
    switch (command.command)
    {
      case tesseract_msgs::EnvironmentCommand::ADD:
      {
        tesseract_scene_graph::Link link = tesseract_rosutils::fromMsg(command.add_link);
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.add_joint);
        if (!visualization_->addLink(link) || !visualization_->addJoint(joint))
          return false;

        if (!env_->addLink(link, joint))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_LINK:
      {
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.move_link_joint);

        std::vector<tesseract_scene_graph::JointConstPtr> joints = env_->getSceneGraph()->getInboundJoints(joint.child_link_name);
        assert(joints.size() == 1);

        if (!visualization_->removeJoint(joints[0]->getName()) || !visualization_->addJoint(joint))
          return false;

        if (!env_->moveLink(joint))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_JOINT:
      {
        if (!visualization_->moveJoint(command.move_joint_name, command.move_joint_parent_link))
          return false;

        if (!env_->moveJoint(command.move_joint_name, command.move_joint_parent_link))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_LINK:
      {
        if (env_->getLink(command.remove_link) == nullptr)
        {
          ROS_WARN("Tried to remove link (%s) that does not exist", command.remove_link.c_str());
          return false;
        }

        std::vector<tesseract_scene_graph::JointConstPtr> joints = env_->getSceneGraph()->getInboundJoints(command.remove_link);
        assert(joints.size() <= 1);

        // get child link names to remove
        std::vector<std::string> child_link_names = env_->getSceneGraph()->getLinkChildrenNames(command.remove_link);

        if (!visualization_->removeLink(command.remove_link))
          return false;

        if (!visualization_->removeJoint(joints[0]->getName()))
          return false;

        for (const auto& link_name : child_link_names)
        {
          if (!visualization_->removeLink(link_name))
            return false;

          std::vector<tesseract_scene_graph::JointConstPtr> joints = env_->getSceneGraph()->getInboundJoints(link_name);
          if (joints.size() == 1)
          {
            if (!visualization_->removeJoint(joints[0]->getName()))
              return false;
          }
        }

        if(!env_->removeLink(command.remove_link))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_JOINT:
      {
        if (!visualization_->removeJoint(command.remove_joint))
          return false;

        if (!env_->removeJoint(command.remove_joint))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_ORIGIN:
      {
        assert(false);
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN:
      {
        assert(false);
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED:
      {
        visualization_->setLinkCollisionEnabled(command.change_link_collision_enabled_name, command.change_link_collision_enabled_value);

        env_->setLinkCollisionEnabled(command.change_link_collision_enabled_name, command.change_link_collision_enabled_value);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY:
      {
        // TODO:: Need to update visualization.
        env_->setLinkVisibility(command.change_link_visibility_name, command.change_link_visibility_value);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION:
      {
        visualization_->addAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2, command.add_allowed_collision.reason);

        env_->addAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2, command.add_allowed_collision.reason);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION:
      {
        visualization_->removeAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2);

        env_->removeAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK:
      {
        visualization_->removeAllowedCollision(command.remove_allowed_collision_link);

        env_->removeAllowedCollision(command.remove_allowed_collision_link);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE:
      {
        return tesseract_rosutils::processMsg(*env_, command.joint_state);
      }
    }
  }
  return true;
}

bool EnvironmentWidget::getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                                       tesseract_msgs::GetEnvironmentChangesResponse& res)
{
  if (req.revision > env_->getRevision())
  {
    res.success = false;
    return false;
  }

  res.id = env_->getName();
  res.revision = env_->getRevision();
  const tesseract_environment::Commands& commands = env_->getCommandHistory();
  for (int i = (req.revision - 1); i < commands.size(); ++i)
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

//void TesseractStateDisplay::setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors)
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

//void TesseractStateDisplay::setLinkColor(const std::string& link_name, const QColor& color)
//{
//  setLinkColor(&state_->getRobot(), link_name, color);
//}

//void TesseractStateDisplay::unsetLinkColor(const std::string& link_name)
//{
//  unsetLinkColor(&state_->getRobot(), link_name);
//}

//void TesseractStateDisplay::setLinkColor(Robot* robot, const std::string& link_name, const QColor& color)
//{
//  RobotLink* link = robot->getLink(link_name);

//  // Check if link exists
//  if (link)
//    link->setColor(
//        static_cast<float>(color.redF()), static_cast<float>(color.greenF()), static_cast<float>(color.blueF()));
//}

//void TesseractStateDisplay::unsetLinkColor(Robot* robot, const std::string& link_name)
//{
//  RobotLink* link = robot->getLink(link_name);

//  // Check if link exists
//  if (link)
//    link->unsetColor();
//}

void EnvironmentWidget::loadEnvironment()
{
  load_env_ = false;
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(urdf_description_property_->getString().toStdString(), urdf_xml_string);
  nh_.getParam(urdf_description_property_->getString().toStdString() + "_semantic", srdf_xml_string);

  // Load URDF model
  if (urdf_xml_string.empty())
  {
    // TODO:
//    setStatus(rviz::StatusProperty::Error, "TesseractState", "No URDF model loaded");
  }
  else
  {
    tesseract_scene_graph::ResourceLocatorFn locator = tesseract_rosutils::locateResource;
    std::pair<tesseract_scene_graph::SceneGraphPtr, tesseract_scene_graph::SRDFModelPtr> data;
    data = tesseract_scene_graph::createSceneGraphFromStrings(urdf_xml_string, srdf_xml_string, locator);
    if (data.first != nullptr && data.second != nullptr)
    {
      bool success = env_->init(data.first);
      assert(success);

      if (success)
      {
        visualization_->clear();
        visualization_->load(env_->getSceneGraph(), true, true, true, true);
        bool oldState = root_link_name_property_->blockSignals(true);
        root_link_name_property_->setStdString(env_->getRootLinkName());
        root_link_name_property_->blockSignals(oldState);
        update_required_ = true;
//        setStatus(rviz::StatusProperty::Ok, "Tesseract", "Tesseract Environment Loaded Successfully");

        changedEnableVisualVisible();
        changedEnableCollisionVisible();
        visualization_->setVisible(true);
      }
      else
      {
//        setStatus(rviz::StatusProperty::Error, "Tesseract", "Tesseract Environment Failed to Load");
      }
    }
    else
    {
//      setStatus(rviz::StatusProperty::Error, "Tesseract", "URDF file failed to parse");
    }
  }

  highlights_.clear();
}

}  // namespace tesseract_rviz
