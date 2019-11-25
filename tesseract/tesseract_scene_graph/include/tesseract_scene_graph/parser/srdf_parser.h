/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author Ioan Sucan */

#ifndef TESSERACT_SCENE_SRDF_PARSER_H
#define TESSERACT_SCENE_SRDF_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <map>
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <tinyxml.h>
#include <console_bridge/console.h>
#include <boost/algorithm/string/trim.hpp>
#include <tesseract_scene_graph/graph.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

/// Main namespace
namespace tesseract_scene_graph
{
/** \brief Representation of semantic information about the robot */
class SRDFModel
{
public:
  using Ptr = std::shared_ptr<SRDFModel>;
  using ConstPtr = std::shared_ptr<const SRDFModel>;

  SRDFModel() = default;
  virtual ~SRDFModel() = default;
  SRDFModel(const SRDFModel&) = default;
  SRDFModel& operator=(const SRDFModel&) = default;
  SRDFModel(SRDFModel&&) = default;
  SRDFModel& operator=(SRDFModel&&) = default;

  /// \brief Load Model from TiXMLElement
  bool initXml(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    clear();
    if (!srdf_xml || srdf_xml->ValueStr() != "robot")
    {
      CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
      return false;
    }

    // get the robot name
    const char* name = srdf_xml->Attribute("name");
    if (!name)
      CONSOLE_BRIDGE_logError("No name given for the robot.");
    else
    {
      name_ = std::string(name);
      boost::trim(name_);
      if (name_ != scene_graph.getName())
        CONSOLE_BRIDGE_logError("Semantic description is not specified for the same robot as the URDF");
    }

    loadVirtualJoints(scene_graph, srdf_xml);
    loadGroups(scene_graph, srdf_xml);
    loadGroupStates(scene_graph, srdf_xml);
    loadEndEffectors(scene_graph, srdf_xml);
    loadLinkSphereApproximations(scene_graph, srdf_xml);
    loadDisabledCollisions(scene_graph, srdf_xml);
    loadPassiveJoints(scene_graph, srdf_xml);

    return true;
  }

  /// \brief Load Model from TiXMLDocument
  bool initXml(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlDocument* srdf_xml)
  {
    TiXmlElement* robot_xml = srdf_xml ? srdf_xml->FirstChildElement("robot") : nullptr;
    if (!robot_xml)
    {
      CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
      return false;
    }
    return initXml(scene_graph, robot_xml);
  }

  /// \brief Load Model given a filename
  bool initFile(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& filename)
  {
    // get the entire file
    std::string xml_string;
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
      while (xml_file.good())
      {
        std::string line;
        std::getline(xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      return initString(scene_graph, xml_string);
    }

    CONSOLE_BRIDGE_logError("Could not open file [%s] for parsing.", filename.c_str());
    return false;
  }

  /// \brief Load Model from a XML-string
  bool initString(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& xmlstring)
  {
    TiXmlDocument xml_doc;
    xml_doc.Parse(xmlstring.c_str());
    if (xml_doc.Error())
    {
      CONSOLE_BRIDGE_logError("Could not parse the SRDF XML File. %s", xml_doc.ErrorDesc());
      return false;
    }

    return initXml(scene_graph, &xml_doc);
  }

  /** \brief A group consists of a set of joints and the
      corresponding descendant links. There are multiple ways to
      specify a group. Directly specifying joints, links or
      chains, or referring to other defined groups. */
  struct Group
  {
    /// The name of the group
    std::string name_;

    /// Directly specified joints to be included in the
    /// group. Descendent links should be implicitly
    /// considered to be part of the group, although this
    /// parsed does not add them to links_. The joints are
    /// checked to be in the corresponding URDF.
    std::vector<std::string> joints_;

    /// Directly specified links to be included in the
    /// group. Parent joints should be implicitly considered
    /// to be part of the group. The links are checked to be
    /// in the corresponding URDF.
    std::vector<std::string> links_;

    /// Specify a chain of links (and the implicit joints) to
    /// be added to the group. Each chain is specified as a
    /// pair of base link and tip link. It is checked that the
    /// chain is indeed a chain in the specified URDF.
    std::vector<std::pair<std::string, std::string> > chains_;

    /// It is sometimes convenient to refer to the content of
    /// another group. A group can include the content of the
    /// referenced groups
    std::vector<std::string> subgroups_;
  };

  /// In addition to the joints specified in the URDF it is
  /// sometimes convenient to add special (virtual) joints. For
  /// example, to connect the robot to the environment in a
  /// meaningful way.
  struct VirtualJoint
  {
    /// The name of the new joint
    std::string name_;

    /// The type of this new joint. This can be "fixed" (0 DOF), "planar" (3 DOF: x,y,yaw) or "floating" (6DOF)
    std::string type_;

    /// The transform applied by this joint to the robot model brings that model to a particular frame.
    std::string parent_frame_;

    /// The link this joint applies to
    std::string child_link_;
  };

  /// Representation of an end effector
  struct EndEffector
  {
    /// The name of the end effector
    std::string name_;

    /// The name of the link this end effector connects to
    std::string parent_link_;

    /// The name of the group to be considered the parent (this group should contain parent_link_)
    /// If not specified, this member is empty.
    std::string parent_group_;

    /// The name of the group that includes the joints & links this end effector consists of
    std::string component_group_;
  };

  /// A named state for a particular group
  struct GroupState
  {
    /// The name of the state
    std::string name_;

    /// The name of the group this state is specified for
    std::string group_;

    /// The values of joints for this state. Each joint can have a value. We use a vector for the 'value' to support
    /// multi-DOF joints
    std::map<std::string, std::vector<double> > joint_values_;
  };

  /// The definition of a sphere
  struct Sphere
  {
    /// The center of the sphere in the link collision frame
    double center_x_;
    double center_y_;
    double center_z_;

    /// The radius of the sphere
    double radius_;
  };

  /// The definition of a list of spheres for a link.
  struct LinkSpheres
  {
    /// The name of the link (as in URDF).
    std::string link_;

    /// The spheres for the link.
    std::vector<Sphere> spheres_;
  };

  /// The definition of a disabled collision between two links
  struct DisabledCollision
  {
    /// The name of the first link (as in URDF) of the disabled collision
    std::string link1_;

    /// The name of the second link (as in URDF) of the disabled collision
    std::string link2_;

    /// The reason why the collision check was disabled
    std::string reason_;
  };

  // Some joints can be passive (not actuated). This structure specifies information about such joints
  struct PassiveJoint
  {
    /// The name of the new joint
    std::string name_;
  };

  /// Get the name of this model
  const std::string& getName() const { return name_; }

  /// Get the list of pairs of links that need not be checked for collisions (because they can never touch given the
  /// geometry and kinematics of the robot)
  const std::vector<DisabledCollision>& getDisabledCollisionPairs() const { return disabled_collisions_; }

  /// Get the list of groups defined for this model
  const std::vector<Group>& getGroups() const { return groups_; }

  /// Get the list of virtual joints defined for this model
  const std::vector<VirtualJoint>& getVirtualJoints() const { return virtual_joints_; }

  /// Get the list of end effectors defined for this model
  const std::vector<EndEffector>& getEndEffectors() const { return end_effectors_; }

  /// Get the list of group states defined for this model
  const std::vector<GroupState>& getGroupStates() const { return group_states_; }

  /// Get the list of known passive joints
  const std::vector<PassiveJoint>& getPassiveJoints() const { return passive_joints_; }

  /// Get the collision spheres list
  const std::vector<LinkSpheres>& getLinkSphereApproximations() const { return link_sphere_approximations_; }

  /// Clear the model
  void clear()
  {
    name_ = "";
    groups_.clear();
    group_states_.clear();
    virtual_joints_.clear();
    end_effectors_.clear();
    link_sphere_approximations_.clear();
    disabled_collisions_.clear();
    passive_joints_.clear();
  }

private:
  void loadVirtualJoints(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    for (TiXmlElement* vj_xml = srdf_xml->FirstChildElement("virtual_joint"); vj_xml;
         vj_xml = vj_xml->NextSiblingElement("virtual_joint"))
    {
      const char* jname = vj_xml->Attribute("name");
      const char* child = vj_xml->Attribute("child_link");
      const char* parent = vj_xml->Attribute("parent_frame");
      const char* type = vj_xml->Attribute("type");
      if (!jname)
      {
        CONSOLE_BRIDGE_logError("Name of virtual joint is not specified");
        continue;
      }
      if (!child)
      {
        CONSOLE_BRIDGE_logError("Child link of virtual joint is not specified");
        continue;
      }
      if (!scene_graph.getLink(boost::trim_copy(std::string(child))))
      {
        CONSOLE_BRIDGE_logError("Virtual joint does not attach to a link on the robot (link '%s' is not known)", child);
        continue;
      }
      if (!parent)
      {
        CONSOLE_BRIDGE_logError("Parent frame of virtual joint is not specified");
        continue;
      }
      if (!type)
      {
        CONSOLE_BRIDGE_logError("Type of virtual joint is not specified");
        continue;
      }
      VirtualJoint vj;
      vj.type_ = std::string(type);
      boost::trim(vj.type_);
      std::transform(vj.type_.begin(), vj.type_.end(), vj.type_.begin(), ::tolower);
      if (vj.type_ != "planar" && vj.type_ != "floating" && vj.type_ != "fixed")
      {
        CONSOLE_BRIDGE_logError("Unknown type of joint: '%s'. Assuming 'fixed' instead. Other known types are 'planar' "
                                "and 'floating'.",
                                type);
        vj.type_ = "fixed";
      }
      vj.name_ = std::string(jname);
      boost::trim(vj.name_);
      vj.child_link_ = std::string(child);
      boost::trim(vj.child_link_);
      vj.parent_frame_ = std::string(parent);
      boost::trim(vj.parent_frame_);
      virtual_joints_.push_back(vj);
    }
  }
  void loadGroups(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    for (TiXmlElement* group_xml = srdf_xml->FirstChildElement("group"); group_xml;
         group_xml = group_xml->NextSiblingElement("group"))
    {
      const char* gname = group_xml->Attribute("name");
      if (!gname)
      {
        CONSOLE_BRIDGE_logError("Group name not specified");
        continue;
      }
      Group g;
      g.name_ = std::string(gname);
      boost::trim(g.name_);

      // get the links in the groups
      for (TiXmlElement* link_xml = group_xml->FirstChildElement("link"); link_xml;
           link_xml = link_xml->NextSiblingElement("link"))
      {
        const char* lname = link_xml->Attribute("name");
        if (!lname)
        {
          CONSOLE_BRIDGE_logError("Link name not specified");
          continue;
        }
        std::string lname_str = boost::trim_copy(std::string(lname));
        if (!scene_graph.getLink(lname_str))
        {
          CONSOLE_BRIDGE_logError(
              "Link '%s' declared as part of group '%s' is not known to the Scene Graph", lname, gname);
          continue;
        }
        g.links_.push_back(lname_str);
      }

      // get the joints in the groups
      for (TiXmlElement* joint_xml = group_xml->FirstChildElement("joint"); joint_xml;
           joint_xml = joint_xml->NextSiblingElement("joint"))
      {
        const char* jname = joint_xml->Attribute("name");
        if (!jname)
        {
          CONSOLE_BRIDGE_logError("Joint name not specified");
          continue;
        }
        std::string jname_str = boost::trim_copy(std::string(jname));
        if (!scene_graph.getJoint(jname_str))
        {
          bool missing = true;
          for (auto& virtual_joint : virtual_joints_)
            if (virtual_joint.name_ == jname_str)
            {
              missing = false;
              break;
            }
          if (missing)
          {
            CONSOLE_BRIDGE_logError(
                "Joint '%s' declared as part of group '%s' is not known to the Scene Graph", jname, gname);
            continue;
          }
        }
        g.joints_.push_back(jname_str);
      }

      // get the chains in the groups
      for (TiXmlElement* chain_xml = group_xml->FirstChildElement("chain"); chain_xml;
           chain_xml = chain_xml->NextSiblingElement("chain"))
      {
        const char* base = chain_xml->Attribute("base_link");
        const char* tip = chain_xml->Attribute("tip_link");
        if (!base)
        {
          CONSOLE_BRIDGE_logError("Base link name not specified for chain");
          continue;
        }
        if (!tip)
        {
          CONSOLE_BRIDGE_logError("Tip link name not specified for chain");
          continue;
        }
        std::string base_str = boost::trim_copy(std::string(base));
        std::string tip_str = boost::trim_copy(std::string(tip));
        if (!scene_graph.getLink(base_str))
        {
          CONSOLE_BRIDGE_logError(
              "Link '%s' declared as part of a chain in group '%s' is not known to the Scene Graph", base, gname);
          continue;
        }
        if (!scene_graph.getLink(tip_str))
        {
          CONSOLE_BRIDGE_logError(
              "Link '%s' declared as part of a chain in group '%s' is not known to the Scene Graph", tip, gname);
          continue;
        }

        g.chains_.emplace_back(base_str, tip_str);
      }

      // get the subgroups in the groups
      for (TiXmlElement* subg_xml = group_xml->FirstChildElement("group"); subg_xml;
           subg_xml = subg_xml->NextSiblingElement("group"))
      {
        const char* sub = subg_xml->Attribute("name");
        if (!sub)
        {
          CONSOLE_BRIDGE_logError("Group name not specified when included as subgroup");
          continue;
        }
        g.subgroups_.push_back(boost::trim_copy(std::string(sub)));
      }
      if (g.links_.empty() && g.joints_.empty() && g.chains_.empty() && g.subgroups_.empty())
        CONSOLE_BRIDGE_logWarn("Group '%s' is empty.", gname);
      groups_.push_back(g);
    }

    // check the subgroups
    std::set<std::string> known_groups;
    bool update = true;
    while (update)
    {
      update = false;
      for (const auto& group : groups_)
      {
        if (known_groups.find(group.name_) != known_groups.end())
          continue;
        if (group.subgroups_.empty())
        {
          known_groups.insert(group.name_);
          update = true;
        }
        else
        {
          bool ok = true;
          for (std::size_t j = 0; ok && j < group.subgroups_.size(); ++j)
            if (known_groups.find(group.subgroups_[j]) == known_groups.end())
              ok = false;
          if (ok)
          {
            known_groups.insert(group.name_);
            update = true;
          }
        }
      }
    }

    // if there are erroneous groups, keep only the valid ones
    if (known_groups.size() != groups_.size())
    {
      std::vector<Group> correct;
      for (const auto& group : groups_)
        if (known_groups.find(group.name_) != known_groups.end())
          correct.push_back(group);
        else
          CONSOLE_BRIDGE_logError("Group '%s' has unsatisfied subgroups", group.name_.c_str());
      groups_.swap(correct);
    }
  }

  void loadGroupStates(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    for (TiXmlElement* gstate_xml = srdf_xml->FirstChildElement("group_state"); gstate_xml;
         gstate_xml = gstate_xml->NextSiblingElement("group_state"))
    {
      const char* sname = gstate_xml->Attribute("name");
      const char* gname = gstate_xml->Attribute("group");
      if (!sname)
      {
        CONSOLE_BRIDGE_logError("Name of group state is not specified");
        continue;
      }
      if (!gname)
      {
        CONSOLE_BRIDGE_logError("Name of group for state '%s' is not specified", sname);
        continue;
      }

      GroupState gs;
      gs.name_ = boost::trim_copy(std::string(sname));
      gs.group_ = boost::trim_copy(std::string(gname));

      bool found = false;
      for (const auto& group : groups_)
        if (group.name_ == gs.group_)
        {
          found = true;
          break;
        }
      if (!found)
      {
        CONSOLE_BRIDGE_logError("Group state '%s' specified for group '%s', but that group is not known", sname, gname);
        continue;
      }

      // get the joint values in the group state
      for (TiXmlElement* joint_xml = gstate_xml->FirstChildElement("joint"); joint_xml;
           joint_xml = joint_xml->NextSiblingElement("joint"))
      {
        const char* jname = joint_xml->Attribute("name");
        const char* jval = joint_xml->Attribute("value");
        if (!jname)
        {
          CONSOLE_BRIDGE_logError("Joint name not specified in group state '%s'", sname);
          continue;
        }
        if (!jval)
        {
          CONSOLE_BRIDGE_logError("Joint name not specified for joint '%s' in group state '%s'", jname, sname);
          continue;
        }
        std::string jname_str = boost::trim_copy(std::string(jname));
        if (!scene_graph.getJoint(jname_str))
        {
          bool missing = true;
          for (const auto& virtual_joint : virtual_joints_)
            if (virtual_joint.name_ == jname_str)
            {
              missing = false;
              break;
            }
          if (missing)
          {
            CONSOLE_BRIDGE_logError(
                "Joint '%s' declared as part of group state '%s' is not known to the URDF", jname, sname);
            continue;
          }
        }
        try
        {
          std::string jval_str = std::string(jval);
          std::istringstream ss(jval_str);
          while (ss.good() && !ss.eof())
          {
            double val;
            ss >> val >> std::ws;
            gs.joint_values_[jname_str].push_back(val);
          }
        }
        catch (const std::invalid_argument& e)
        {
          CONSOLE_BRIDGE_logError("Unable to parse joint value '%s'", jval);
        }
        catch (const std::out_of_range& e)
        {
          CONSOLE_BRIDGE_logError("Unable to parse joint value '%s' (out of range)", jval);
        }

        if (gs.joint_values_.empty())
          CONSOLE_BRIDGE_logError(
              "Unable to parse joint value ('%s') for joint '%s' in group state '%s'", jval, jname, sname);
      }
      group_states_.push_back(gs);
    }
  }

  void loadEndEffectors(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    for (TiXmlElement* eef_xml = srdf_xml->FirstChildElement("end_effector"); eef_xml;
         eef_xml = eef_xml->NextSiblingElement("end_effector"))
    {
      const char* ename = eef_xml->Attribute("name");
      const char* gname = eef_xml->Attribute("group");
      const char* parent = eef_xml->Attribute("parent_link");
      const char* parent_group = eef_xml->Attribute("parent_group");
      if (!ename)
      {
        CONSOLE_BRIDGE_logError("Name of end effector is not specified");
        continue;
      }
      if (!gname)
      {
        CONSOLE_BRIDGE_logError("Group not specified for end effector '%s'", ename);
        continue;
      }
      EndEffector e;
      e.name_ = std::string(ename);
      boost::trim(e.name_);
      e.component_group_ = std::string(gname);
      boost::trim(e.component_group_);
      bool found = false;
      for (const auto& group : groups_)
        if (group.name_ == e.component_group_)
        {
          found = true;
          break;
        }
      if (!found)
      {
        CONSOLE_BRIDGE_logError(
            "End effector '%s' specified for group '%s', but that group is not known", ename, gname);
        continue;
      }
      if (!parent)
      {
        CONSOLE_BRIDGE_logError("Parent link not specified for end effector '%s'", ename);
        continue;
      }
      e.parent_link_ = std::string(parent);
      boost::trim(e.parent_link_);
      if (!scene_graph.getLink(e.parent_link_))
      {
        CONSOLE_BRIDGE_logError(
            "Link '%s' specified as parent for end effector '%s' is not known to the URDF", parent, ename);
        continue;
      }
      if (parent_group)
      {
        e.parent_group_ = std::string(parent_group);
        boost::trim(e.parent_group_);
      }
      end_effectors_.push_back(e);
    }
  }
  void loadLinkSphereApproximations(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    for (TiXmlElement* cslink_xml = srdf_xml->FirstChildElement("link_sphere_approximation"); cslink_xml;
         cslink_xml = cslink_xml->NextSiblingElement("link_sphere_approximation"))
    {
      int non_0_radius_sphere_cnt = 0;
      const char* link_name = cslink_xml->Attribute("link");
      if (!link_name)
      {
        CONSOLE_BRIDGE_logError("Name of link is not specified in link_collision_spheres");
        continue;
      }

      LinkSpheres link_spheres;
      link_spheres.link_ = boost::trim_copy(std::string(link_name));
      if (!scene_graph.getLink(link_spheres.link_))
      {
        CONSOLE_BRIDGE_logError("Link '%s' is not known to URDF.", link_name);
        continue;
      }

      // get the spheres for this link
      int cnt = 0;
      for (TiXmlElement* sphere_xml = cslink_xml->FirstChildElement("sphere"); sphere_xml;
           sphere_xml = sphere_xml->NextSiblingElement("sphere"), cnt++)
      {
        const char* s_center = sphere_xml->Attribute("center");
        const char* s_r = sphere_xml->Attribute("radius");
        if (!s_center || !s_r)
        {
          CONSOLE_BRIDGE_logError(
              "Link collision sphere %d for link '%s' does not have both center and radius.", cnt, link_name);
          continue;
        }

        Sphere sphere;
        try
        {
          std::stringstream center(s_center);
          center.exceptions(std::stringstream::failbit | std::stringstream::badbit);
          center >> sphere.center_x_ >> sphere.center_y_ >> sphere.center_z_;
          sphere.radius_ = std::stod(s_r);
        }
        catch (std::stringstream::failure& e)
        {
          CONSOLE_BRIDGE_logError(
              "Link collision sphere %d for link '%s' has bad center attribute value.", cnt, link_name);
          continue;
        }
        catch (const std::invalid_argument& e)
        {
          CONSOLE_BRIDGE_logError(
              "Link collision sphere %d for link '%s' has bad radius attribute value.", cnt, link_name);
          continue;
        }
        catch (const std::out_of_range& e)
        {
          CONSOLE_BRIDGE_logError(
              "Link collision sphere %d for link '%s' has an out of range radius attribute value.", cnt, link_name);
          continue;
        }

        // ignore radius==0 spheres unless there is only 1 of them
        //
        // NOTE:
        //  - If a link has no sphere_approximation then one will be generated
        //     automatically containing a single sphere which encloses the entire
        //     collision geometry.  Internally this is represented by not having
        //     a link_sphere_approximations_ entry for this link.
        //  - If a link has only spheres with radius 0 then it will not be
        //     considered for collision detection.  In this case the internal
        //     representation is a single radius=0 sphere.
        //  - If a link has at least one sphere with radius>0 then those spheres
        //     are used.  Any radius=0 spheres are eliminated.
        if (sphere.radius_ > std::numeric_limits<double>::epsilon())
        {
          if (non_0_radius_sphere_cnt == 0)
            link_spheres.spheres_.clear();
          link_spheres.spheres_.push_back(sphere);
          non_0_radius_sphere_cnt++;
        }
        else if (non_0_radius_sphere_cnt == 0)
        {
          sphere.center_x_ = 0.0;
          sphere.center_y_ = 0.0;
          sphere.center_z_ = 0.0;
          sphere.radius_ = 0.0;
          link_spheres.spheres_.clear();
          link_spheres.spheres_.push_back(sphere);
        }
      }

      if (!link_spheres.spheres_.empty())
        link_sphere_approximations_.push_back(link_spheres);
    }
  }

  void loadDisabledCollisions(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    for (TiXmlElement* c_xml = srdf_xml->FirstChildElement("disable_collisions"); c_xml;
         c_xml = c_xml->NextSiblingElement("disable_collisions"))
    {
      const char* link1 = c_xml->Attribute("link1");
      const char* link2 = c_xml->Attribute("link2");
      if (!link1 || !link2)
      {
        CONSOLE_BRIDGE_logError("A pair of links needs to be specified to disable collisions");
        continue;
      }
      DisabledCollision dc;
      dc.link1_ = boost::trim_copy(std::string(link1));
      dc.link2_ = boost::trim_copy(std::string(link2));
      if (!scene_graph.getLink(dc.link1_))
      {
        CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link1);
        continue;
      }
      if (!scene_graph.getLink(dc.link2_))
      {
        CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link2);
        continue;
      }
      const char* reason = c_xml->Attribute("reason");
      if (reason)
        dc.reason_ = std::string(reason);
      disabled_collisions_.push_back(dc);
    }
  }

  void loadPassiveJoints(const tesseract_scene_graph::SceneGraph& scene_graph, TiXmlElement* srdf_xml)
  {
    for (TiXmlElement* c_xml = srdf_xml->FirstChildElement("passive_joint"); c_xml;
         c_xml = c_xml->NextSiblingElement("passive_joint"))
    {
      const char* name = c_xml->Attribute("name");
      if (!name)
      {
        CONSOLE_BRIDGE_logError("No name specified for passive joint. Ignoring.");
        continue;
      }
      PassiveJoint pj;
      pj.name_ = boost::trim_copy(std::string(name));

      // see if a virtual joint was marked as passive
      bool vjoint = false;
      for (std::size_t i = 0; !vjoint && i < virtual_joints_.size(); ++i)
        if (virtual_joints_[i].name_ == pj.name_)
          vjoint = true;

      if (!vjoint && !scene_graph.getJoint(pj.name_))
      {
        CONSOLE_BRIDGE_logError("Joint '%s' marked as passive is not known to the URDF. Ignoring.", name);
        continue;
      }
      passive_joints_.push_back(pj);
    }
  }

  std::string name_;
  std::vector<Group> groups_;
  std::vector<GroupState> group_states_;
  std::vector<VirtualJoint> virtual_joints_;
  std::vector<EndEffector> end_effectors_;
  std::vector<LinkSpheres> link_sphere_approximations_;
  std::vector<DisabledCollision> disabled_collisions_;
  std::vector<PassiveJoint> passive_joints_;
};

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_SRDF_PARSER_H
