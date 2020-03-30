
#include "ui_trajectory_preview_widget.h"
#include <tesseract_rviz/render_tools/trajectory_preview_widget.h>
#include <tesseract_rviz/render_tools/robot_trajectory.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tesseract_msgs/Trajectory.h>
#include <QObject>

namespace tesseract_rviz
{
const static double TARGET_FPS = 15.0;

class TrajectoryPreviewImpl : public QObject
{
  Q_OBJECT
public:
  TrajectoryPreviewImpl() : QObject(), scale_(1.0) {}

  void initialize(const std::string& input_traj_topic, const std::string& output_state_topic)
  {
    ros::NodeHandle nh;

    traj_sub_ = nh.subscribe(input_traj_topic, 1, &TrajectoryPreviewImpl::onNewTrajectory, this);
    state_pub_ = nh.advertise<sensor_msgs::JointState>(output_state_topic, 1, true);
    timer_ = nh.createTimer(ros::Rate(TARGET_FPS), &TrajectoryPreviewImpl::onAnimateCallback, this, false, false);
  }

  void onNewTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& msg)
  {
    stopDisplay();

    // Prepare the new trajectory message
    display_traj_.reset(new RobotTrajectory(*msg));

    trajectory_duration_ = display_traj_->getWayPointDurationFromStart(display_traj_->getWayPointCount() - 1);

    // Let listeners know that a new trajectory is ready
    emit newTrajectory(trajectory_duration_);

    // Reset state associated with trajectory playback
    current_time_ = 0.0;

    startDisplay();
  }

  void onAnimateCallback(const ros::TimerEvent& e)
  {
    if (!display_traj_)
      return;

    auto step = (e.current_real - e.last_real).toSec() * scale_;
    if (e.last_real == ros::Time())  // Last time is zero if the timer gets restarted
    {
      step = 0;
    }

    // compute current time
    current_time_ = current_time_ + step;

    if (current_time_ < trajectory_duration_)
    {
      publishStateAtTime(current_time_);
      emit update(current_time_ / trajectory_duration_);
    }
    else
    {
      // Make sure we always publish the last state even if we've gone over the end of the trajectory
      publishStateAtTime(trajectory_duration_);
      emit update(1.0);
      emit trajectoryFinished();
      stopDisplay();
    }
  }

  void publishStateAtTime(double t)
  {
    if (!display_traj_)
      return;

    sensor_msgs::JointState state_msg;

    // Compute the interpolated state
    display_traj_->getStateAtDurationFromStart(t, state_msg);

    // Display
    state_pub_.publish(state_msg);
  }

  void setCurrentTime(int index, int num_indexes)
  {
    if (!display_traj_)
      return;

    const auto total_time = trajectory_duration_;
    current_time_ = total_time * static_cast<double>(index) / static_cast<double>(num_indexes);
    publishStateAtTime(current_time_);
  }

  double currentTrajectoryDuration() const { return trajectory_duration_; }

Q_SIGNALS:
  void newTrajectory(double total_time);
  void trajectoryFinished();
  void update(double ratio);

private:
  ros::Subscriber traj_sub_;
  ros::Publisher state_pub_;
  ros::Timer timer_;

  double trajectory_duration_;
  double current_time_;
  double scale_;

  RobotTrajectory::Ptr display_traj_;
};

TrajectoryPreviewWidget::TrajectoryPreviewWidget(QWidget* parent) : QWidget(parent)
{
  ui_ = new Ui::TrajectoryPreview();
  ui_->setupUi(this);

  num_ticks_ = ui_->timeSlider->maximum() - ui_->timeSlider->minimum();

  // QT Events
  connect(ui_->playButton, SIGNAL(clicked()), this, SLOT(onPlayPauseButton()));
  connect(ui_->scaleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onScaleChanged(double)));
  connect(ui_->timeSlider, SIGNAL(sliderMoved(int)), this, SLOT(onSliderChanged(int)));

  //  // Create ROS stuff
  //  impl_ = new TrajectoryPreviewImpl();
  //  connect(impl_, SIGNAL(newTrajectory(double)), this, SLOT(onNewTrajectory(double)));
  //  connect(impl_, SIGNAL(update(double)), this, SLOT(onUpdate(double)));
  //  connect(impl_, SIGNAL(trajectoryFinished()), this, SLOT(onTrajectoryFinished()));
}

void TrajectoryPreviewWidget::onPlayPauseButton()
{
  if (ui_->playButton->isChecked())
  {
    startDisplay();
    ui_->playButton->setText(tr("Pause"));
  }
  else
  {
    stopDisplay();
    ui_->playButton->setText(tr("Play"));
  }
}

void TrajectoryPreviewWidget::onScaleChanged(double new_scale) { scale_ = new_scale; }

void TrajectoryPreviewWidget::onSliderChanged(int new_position) { impl_->setCurrentTime(new_position, num_ticks_); }

void TrajectoryPreviewWidget::onNewTrajectory(double total_duration)
{
  ui_->timeSlider->setValue(ui_->timeSlider->minimum());
  ui_->playButton->setChecked(true);
  ui_->playButton->setText(tr("Pause"));
  ui_->totalLabel->setText(QString::number(total_duration, 'g', 4));
}

void TrajectoryPreviewWidget::onTrajectoryFinished()
{
  ui_->playButton->setChecked(false);
  ui_->playButton->setText(tr("Play"));
}

void TrajectoryPreviewWidget::onUpdate(double ratio)
{
  int tick = static_cast<int>(ratio * num_ticks_ + 0.5);
  ui_->timeSlider->setValue(tick);
  ui_->currentLabel->setText(QString::number(ratio * impl_->currentTrajectoryDuration(), 'g', 4));
}

void TrajectoryPreviewWidget::initializeROS(const std::string& input_traj_topic, const std::string& output_state_topic)
{
  impl_->initialize(input_traj_topic, output_state_topic);
}

void TrajectoryPreviewWidget::setTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  impl_->onNewTrajectory(boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory));
}

void TrajectoryPreviewWidget::startDisplay()
{
  if (!display_traj_)
    return;

  if (current_time_ > 0.999 * trajectory_duration_)
  {
    // reset timer
    current_time_ = 0.0;
  }
  timer_.start();
}

void TrajectoryPreviewWidget::stopDisplay() { timer_.stop(); }

}  // namespace tesseract_rviz
