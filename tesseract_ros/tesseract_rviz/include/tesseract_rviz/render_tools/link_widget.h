/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TESSERACT_RVIZ_ROBOT_LINK_H
#define TESSERACT_RVIZ_ROBOT_LINK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

#include <string>
#include <map>

#include <QObject>

#ifndef Q_MOC_RUN
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include <Eigen/Eigen>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/link.h>
#endif

#include <rviz/ogre_helpers/object.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/selection/forwards.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace Ogre
{
class SceneManager;
class Entity;
class SubEntity;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
class RibbonTrail;
}  // namespace Ogre

namespace rviz
{
class Shape;
class Axes;
class DisplayContext;
class FloatProperty;
class Property;
class BoolProperty;
class QuaternionProperty;
class VectorProperty;
class StringProperty;
}  // namespace rviz

namespace octomap
{
class OcTree;
}

namespace tesseract_rviz
{
class VisualizationWidget;
class EnvLinkSelectionHandler;
class JointWidget;
using EnvLinkSelectionHandlerPtr = std::shared_ptr<EnvLinkSelectionHandler>;

enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
};

/**
 * @brief The EnvLink class
 *
 * A link has visualization of a link: Start, Trajectory and End. These can all be shown at once or individually.
 */
class LinkWidget : public QObject
{
  Q_OBJECT
public:
  LinkWidget(VisualizationWidget* env, const tesseract_scene_graph::Link& link, bool visual, bool collision);

  virtual ~LinkWidget();

  virtual void setAlpha(float a);

  /**
   * @brief Set the current state visualization link transform
   * @param transform
   */
  virtual void setCurrentTransform(const Eigen::Isometry3d& transform);

  /**
   * @brief Set the start state visualization link transform
   * @param transform
   */
  virtual void setStartTransform(const Eigen::Isometry3d& transform);

  /**
   * @brief Set the end state visualization link transform
   * @param transform
   */
  virtual void setEndTransform(const Eigen::Isometry3d& transform);

  /**
   * @brief Set trajectory for the link
   * @param trajectory
   */
  virtual void setTrajectory(const std::vector<Eigen::Isometry3d>& trajectory);

  /** @brief Hide trajectory */
  virtual void clearTrajectory();

  // This is usefule when wanting to simulate the trajectory
  virtual void showTrajectoryWaypointOnly(int waypoint);

  // access
  const std::string& getName() const { return name_; }

  rviz::Property* getLinkProperty() const { return link_property_; }

  Ogre::SceneNode* getCurrentVisualNode() const { return visual_current_node_; }
  Ogre::SceneNode* getCurrentCollisionNode() const { return collision_current_node_; }

  Ogre::SceneNode* getStartVisualNode() const { return visual_start_node_; }
  Ogre::SceneNode* getStartCollisionNode() const { return collision_start_node_; }

  Ogre::SceneNode* getEndVisualNode() const { return visual_end_node_; }
  Ogre::SceneNode* getEndCollisionNode() const { return collision_end_node_; }

  VisualizationWidget* getEnvVisualization() const { return env_; }
  // Remove link_property_ from its old parent and add to new_parent.  If new_parent==nullptr then leav unparented.
  void setParentProperty(rviz::Property* new_parent);

  // hide or show all sub properties (hide to make tree easier to see)
  virtual void hideSubProperties(bool hide);

  void setToErrorMaterial();
  void setToNormalMaterial();

  void setColor(float red, float green, float blue);
  void unsetColor();

  /// set whether the link is selectable.  If false objects behind/inside the link can be selected/manipulated.  Returns
  /// old value.
  bool setSelectable(bool selectable);
  bool getSelectable();

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  void setCollisionEnabled(bool enabled);
  void addAllowedCollision(const std::string& link_name, const std::string& reason);
  void removeAllowedCollision(const std::string& link_name);
  void clearAllowedCollisions();

  bool hasGeometry() const;

  /* If set to true, the link will only render to the depth channel
   * and be in render group 0, so it is rendered before anything else.
   * Thus, it will occlude other objects without being visible.
   */
  void setOnlyRenderDepth(bool onlyRenderDepth);
  bool getOnlyRenderDepth() const { return only_render_depth_; }
  // place subproperties as children of details_ or joint_property_
  void useDetailProperty(bool use_detail);

  // expand all sub properties
  void expandDetails(bool expand);

  void setLinkPropertyDescription();

public Q_SLOTS:
  /** @brief Update the visibility of the link elements: visual mesh, collision mesh, trail, and axes.
   *
   * Called by Robot when changing visual and collision visibilities,
   * since each link may be enabled or disabled. */
  void updateVisibility();

private Q_SLOTS:
  void updateAlpha();
  void updateTrail();
  void updateAxes();

private:
  void setRenderQueueGroup(Ogre::uint8 group);
  bool getEnabled() const;

  Ogre::Entity* createEntityForMeshData(const std::string& entity_name,
                                        const std::shared_ptr<const tesseract_common::VectorVector3d>& mesh_vertices,
                                        const std::shared_ptr<const Eigen::VectorXi>& mesh_faces);

  bool createEntityForGeometryElement(const tesseract_scene_graph::Link& link,
                                      const tesseract_geometry::Geometry& geom,
                                      const Eigen::Isometry3d& origin,
                                      const std::string& material_name,
                                      bool isVisual);

  void createVisual(const tesseract_scene_graph::Link& link);
  void createCollision(const tesseract_scene_graph::Link& link);

  void createSelection();
  Ogre::MaterialPtr getMaterialForLink(const tesseract_scene_graph::Link& link, const std::string& material_name = "");

  void setOctomapColor(double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point* point);

  void clone(Ogre::SceneNode* scene_node,
             Ogre::SceneNode* cloned_scene_node,
             std::vector<Ogre::Entity*>& meshes,
             std::vector<rviz::PointCloud*>& octrees);

protected:
  VisualizationWidget* env_;
  Ogre::SceneManager* scene_manager_;
  rviz::DisplayContext* context_;

  std::string name_;  ///< Name of this link

  // properties
  rviz::Property* link_property_;
  rviz::Property* details_;
  rviz::VectorProperty* position_property_;
  rviz::QuaternionProperty* orientation_property_;
  rviz::Property* trail_property_;
  rviz::Property* axes_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::StringProperty* collision_enabled_property_;
  rviz::Property* allowed_collision_matrix_property_;

private:
  typedef std::map<Ogre::SubEntity*, Ogre::MaterialPtr> M_SubEntityToMaterial;
  M_SubEntityToMaterial materials_;
  Ogre::MaterialPtr default_material_;
  std::string default_material_name_;
  std::map<std::string, rviz::StringProperty*> acm_;

  std::vector<Ogre::Entity*> visual_current_meshes_;     ///< The entities representing the visual mesh of this link (if
                                                         ///< they
                                                         /// exist)
  std::vector<Ogre::Entity*> collision_current_meshes_;  ///< The entities representing the collision mesh of this link
                                                         ///< (if they
                                                         /// exist)

  std::vector<Ogre::Entity*> visual_start_meshes_;  ///< The entities representing the visual mesh of this link (if they
                                                    /// exist)
  std::vector<Ogre::Entity*> collision_start_meshes_;  ///< The entities representing the collision mesh of this link
                                                       ///< (if they
                                                       /// exist)

  std::vector<Ogre::Entity*> visual_trajectory_meshes_;  ///< The entities representing the visual mesh of this link (if
                                                         ///< they
                                                         /// exist)
  std::vector<Ogre::Entity*> collision_trajectory_meshes_;  ///< The entities representing the collision mesh of this
                                                            ///< link (if they
                                                            /// exist)

  std::vector<Ogre::Entity*> visual_end_meshes_;  ///< The entities representing the visual mesh of this link (if they
                                                  /// exist)
  std::vector<Ogre::Entity*> collision_end_meshes_;  ///< The entities representing the collision mesh of this link (if
                                                     ///< they
                                                     /// exist)

  struct OctreeDataContainer
  {
    rviz::PointCloud* point_cloud;
    std::vector<rviz::PointCloud::Point> points;
    float size;

    rviz::PointCloud* clone();
  };

  std::vector<OctreeDataContainer> visual_current_octrees_;     ///< The object representing the visual of this link (if
                                                                ///< they exist)
  std::vector<OctreeDataContainer> collision_current_octrees_;  ///< The object representing the visual of this link (if
                                                                ///< they
                                                                /// exist)

  std::vector<rviz::PointCloud*> visual_start_octrees_;  ///< The object representing the visual of this link (if they
                                                         ///< exist)
  std::vector<rviz::PointCloud*> collision_start_octrees_;  ///< The object representing the visual of this link (if
                                                            ///< they
                                                            /// exist)

  std::vector<rviz::PointCloud*> visual_trajectory_octrees_;  ///< The object representing the visual of this link (if
                                                              ///< they exist)
  std::vector<rviz::PointCloud*> collision_trajectory_octrees_;  ///< The object representing the visual of this link
                                                                 ///< (if they
                                                                 /// exist)

  std::vector<rviz::PointCloud*> visual_end_octrees_;     ///< The object representing the visual of this link (if they
                                                          ///< exist)
  std::vector<rviz::PointCloud*> collision_end_octrees_;  ///< The object representing the visual of this link (if they
                                                          /// exist)

  Ogre::SceneNode* visual_current_node_;     ///< The scene node the visual meshes are attached to
  Ogre::SceneNode* collision_current_node_;  ///< The scene node the collision meshes are attached to
  Ogre::SceneNode* visual_start_node_;       ///< The scene node the visual meshes are attached to
  Ogre::SceneNode* collision_start_node_;    ///< The scene node the collision meshes are attached to
  Ogre::SceneNode* visual_end_node_;         ///< The scene node the visual meshes are attached to
  Ogre::SceneNode* collision_end_node_;      ///< The scene node the collision meshes are attached to
  Ogre::SceneNode* visual_trajectory_node_;
  Ogre::SceneNode* collision_trajectory_node_;
  std::vector<Ogre::SceneNode*> visual_trajectory_waypoint_nodes_;
  std::vector<bool> visual_trajectory_waypoint_visibility_;  // cache visibility state for toggling collision/visual
                                                             // visibility
  std::vector<Ogre::SceneNode*> collision_trajectory_waypoint_nodes_;
  std::vector<bool> collision_trajectory_waypoint_visibility_;  // cache visibility state for toggling collision/visual
                                                                // visibility

  Ogre::RibbonTrail* trail_;

  rviz::Axes* axes_;

  float material_alpha_;  ///< If material is not a texture, this saves the alpha value set in the URDF, otherwise is
                          /// 1.0.
  float alpha_;           ///< Alpha value from top-level robot alpha Property (set via setRobotAlpha()).

  bool only_render_depth_;
  bool is_selectable_;

  // joint stuff
  std::string joint_name_;

  EnvLinkSelectionHandlerPtr selection_handler_;

  Ogre::MaterialPtr color_material_;
  bool using_color_;

  friend class EnvLinkSelectionHandler;
};

}  // namespace tesseract_rviz

#endif  // TESSSERACT_RVIZ_ROBOT_LINK_H
