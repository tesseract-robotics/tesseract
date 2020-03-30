#ifndef TESSERACT_RVIZ_TRAJECTORY_PREVIEW_WIDGET_H
#define TESSERACT_RVIZ_TRAJECTORY_PREVIEW_WIDGET_H

#include <tesseract_common/macros.h>

#ifndef Q_MOC_RUN
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract/tesseract.h>
#include <sensor_msgs/JointState.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#endif

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <QWidget>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace Ui
{
class TrajectoryPreview;
}

namespace tesseract_rviz
{
class TrajectoryPreviewImpl;

class TrajectoryPreviewWidget : public QWidget
{
  Q_OBJECT
public:
  using Ptr = std::shared_ptr<TrajectoryPreviewWidget>;
  using ConstPtr = std::shared_ptr<const TrajectoryPreviewWidget>;

  TrajectoryPreviewWidget(QWidget* parent = 0);
  ~TrajectoryPreviewWidget() = default;

  /**
   * @brief Creates a ROS subscriber to a trajectory topic
   * and a publisher for the state of the robot
   * @param input_traj_topic
   * @param output_state_topic
   */
  void initializeROS(const std::string& input_traj_topic, const std::string& output_state_topic);

  /**
   * @brief Programmatically sets the displayed trajectory (rather than waiting for one to
   * be published)
   * @param trajectory
   */
  void setTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
  double currentTrajectoryDuration() const;

  Q_SLOTS:
  // Chnage the topics
  void onTrajectoryTopicChanged(const QString& input_traj_topic);
  void onStateTopicChanged(const QString& output_state_topic);

Q_SIGNALS:
  void newTrajectory(double total_time);
  void trajectoryFinished();
  void update(double ratio);

protected Q_SLOTS:
  // User Interactions
  void onPlayPauseButton();
  void onScaleChanged(double new_scale);
  void onSliderChanged(int new_position);

  // Automatic ROS interactions
  void onNewTrajectory(double total_duration);
  void onTrajectoryFinished();
  void onUpdate(double ratio);

  void onStopDisplay();
  void onStartDisplay();
  void onAnimateCallback(const ros::TimerEvent& e);

  void onPublishStateAtTime(double t);
  void onSetScale(double new_scale);
  void onSetCurrentTime(int index, int num_indexes);

private:
  ros::NodeHandle nh_;
  Ui::TrajectoryPreview* ui_;

  ros::Subscriber traj_sub_;
  ros::Publisher state_pub_;
  ros::Timer timer_;

  double trajectory_duration_;
  double current_time_;
  double scale_{ 1.0 };
  int num_ticks_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_TRAJECTORY_PREVIEW_WIDGET_H
