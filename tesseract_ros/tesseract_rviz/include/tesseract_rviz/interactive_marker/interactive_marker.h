/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#ifndef TESSERACT_RVIZ_INTERACTIVE_MARKER_INTERACTIVE_MARKER_H
#define TESSERACT_RVIZ_INTERACTIVE_MARKER_INTERACTIVE_MARKER_H

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <Eigen/Geometry>

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <tesseract_rviz/interactive_marker/interactive_marker_control.h>
#endif

#include "rviz/selection/forwards.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/properties/status_property.h"

namespace Ogre
{
class SceneNode;
}

class QMenu;

namespace rviz
{
class DisplayContext;
// class InteractiveMarkerDisplay;
}  // namespace rviz
namespace tesseract_rviz
{
class InteractiveMarker : public QObject
{
  Q_OBJECT
public:
  using Ptr = boost::shared_ptr<InteractiveMarker>;
  using ConstPtr = boost::shared_ptr<const InteractiveMarker>;

  InteractiveMarker(std::string name,
                    std::string description,
                    std::string reference_frame,
                    Ogre::SceneNode* scene_node,
                    rviz::DisplayContext* context,
                    bool reference_frame_locked,
                    float scale = 1);
  virtual ~InteractiveMarker();

  InteractiveMarkerControl::Ptr createInteractiveControl(const std::string& name,
                                                         const std::string& description,
                                                         const InteractiveMode interactive_mode,
                                                         const OrientationMode orientation_mode,
                                                         const bool always_visible,
                                                         const Ogre::Quaternion& orientation);

  // called every frame update
  void update(float wall_dt);

  // directly set the pose, relative to parent frame
  // if publish is set to true, publish the change
  void setPose(Ogre::Vector3 position, Ogre::Quaternion orientation, const std::string& control_name);

  void translate(Ogre::Vector3 delta_position, const std::string& control_name);
  void rotate(Ogre::Quaternion delta_orientation, const std::string& control_name);

  // schedule a pose reset once dragging is finished
  void requestPoseUpdate(Ogre::Vector3 position, Ogre::Quaternion orientation);

  void startDragging();
  void stopDragging();

  const Ogre::Vector3& getPosition() { return position_; }
  const Ogre::Quaternion& getOrientation() { return orientation_; }

  void setSize(float scale);
  float getSize() { return scale_; }
  const std::string& getReferenceFrame() { return reference_frame_; }
  const std::string& getName() { return name_; }

  /**
   * @brief Set visibility
   * @param visible
   */
  void setVisible(bool visible);

  // show name above marker
  void setShowDescription(bool show);

  // show axes in origin
  void setShowAxes(bool show);

  // show visual helpers while dragging
  void setShowVisualAids(bool show);

  // @return true if the mouse event was intercepted, false if it was ignored
  bool handleMouseEvent(rviz::ViewportMouseEvent& event, const std::string& control_name);

  /**
   * Supports selection and menu events from a 3D cursor.
   *
   * @param  event        A struct holding certain event data (see full description
   * InteractiveMarkerControl::handle3DCursorEvent)
   * @param  cursor_pos   The world-relative position of the 3D cursor.
   * @param  cursor_rot   The world-relative orientation of the 3D cursor.
   * @param  control_name The name of the child InteractiveMarkerControl calling this function.
   * @return              true if the cursor event was intercepted, false if it was ignored
   */
  bool handle3DCursorEvent(rviz::ViewportMouseEvent& event,
                           const Ogre::Vector3& cursor_pos,
                           const Ogre::Quaternion& cursor_rot,
                           const std::string& control_name);

  /**
   * Pop up the context menu for this marker.
   *
   * @param  event         A struct holding certain event data (see full description on
   * InteractiveMarkerControl::handle3DCursorEvent)
   * @param  control_name  The name of the InteractiveMarkerControl that was selected.
   * @param  three_d_point The world-relative position associated with this mouse-click or cursor event.
   * @param  valid_point   True if three_d_point is valid (e.g. if the mouse ray was successfully intersected with
   * marker geometry).
   */
  void showMenu(rviz::ViewportMouseEvent& event,
                const std::string& control_name,
                const Ogre::Vector3& three_d_point,
                bool valid_point);

  // Emit pose pose information
  void publishFeedback(bool mouse_point_valid = false,
                       const Ogre::Vector3& mouse_point_rel_world = Ogre::Vector3(0, 0, 0));

  bool hasMenu() { return has_menu_; }

  /** @return A shared_ptr to the QMenu owned by this InteractiveMarker. */
  boost::shared_ptr<QMenu> getMenu() { return menu_; }

Q_SIGNALS:

  void userFeedback(std::string reference_frame,
                    Eigen::Isometry3d transform,
                    Eigen::Vector3d mouse_point,
                    bool mouse_point_valid);
  void statusUpdate(rviz::StatusProperty::Level level, const std::string& name, const std::string& text);

protected Q_SLOTS:
  void handleMenuSelect(int menu_item_id);

protected:
  void publishPose();

  void reset();

  // set the pose of the parent frame, relative to the fixed frame
  void updateReferencePose();

  QString makeMenuString(const std::string& entry);

  // Recursively append menu and submenu entries to menu, based on a
  // vector of menu entry id numbers describing the menu entries at the
  // current level.
  void populateMenu(QMenu* menu, std::vector<uint32_t>& ids);

  // Update the visibility based on visible and show_description variable
  void updateDescriptionVisibility();

  // Update the visibility based on visible and show_axis variable
  void updateAxesVisibility();

  // Update the visibility based on visible and show_visual_aids variable
  void updateVisualAidsVisibility();

  bool visible_;
  rviz::DisplayContext* context_;

  // pose of parent coordinate frame
  std::string reference_frame_;
  bool reference_frame_locked_;

  // node representing reference frame in tf, like /map, /base_link, /head, etc.
  Ogre::SceneNode* reference_node_;

  // pose being controlled, relative to reference frame
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  // has the pose changed since the last feedback was sent?
  bool pose_changed_;
  double time_since_last_feedback_;

  std::map<std::string, InteractiveMarkerControl::Ptr> controls_;

  std::string name_;
  std::string description_;

  bool dragging_;

  // pose being controlled
  bool pose_update_requested_;
  Ogre::Vector3 requested_position_;
  Ogre::Quaternion requested_orientation_;

  float scale_;

  boost::shared_ptr<QMenu> menu_;
  bool has_menu_;

  // Helper to more simply represent the menu tree.
  struct MenuNode
  {
    //    visualization_msgs::MenuEntry entry;
    std::vector<uint32_t> child_ids;
  };

  // maps menu index to menu entry and item
  std::map<uint32_t, MenuNode> menu_entries_;

  // Helper to store the top level of the menu tree.
  std::vector<uint32_t> top_level_menu_ids_;

  // which control has popped up the menu
  std::string last_control_name_;

  // visual aids

  rviz::Axes* axes_;

  InteractiveMarkerControl::Ptr description_control_;

  std::string topic_ns_;
  std::string client_id_;

  boost::recursive_mutex mutex_;

  boost::shared_ptr<boost::thread> sys_thread_;

  bool got_3d_point_for_menu_;
  Ogre::Vector3 three_d_point_for_menu_;

  bool show_visual_aids_;
  bool show_axes_;
  bool show_description_;
};

}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_INTERACTIVE_MARKER_H
