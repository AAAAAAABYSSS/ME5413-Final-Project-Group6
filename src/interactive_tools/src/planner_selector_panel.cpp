#include <interactive_tools/planner_selector_panel.h>
#include "ui_planner_selector_panel.h"

namespace interactive_tools {

PlannerSelectorPanel::PlannerSelectorPanel(QWidget* parent)
  : rviz::Panel(parent), ui_(new Ui::PlannerSelectorPanel) {
  ui_->setupUi(this);

  global_pub_ = nh_.advertise<std_msgs::String>("/planner_selector/global_planner", 1);
  local_pub_ = nh_.advertise<std_msgs::String>("/planner_selector/local_planner", 1);
  restart_client_ = nh_.serviceClient<std_srvs::Trigger>("/restart_move_base");

  ui_->globalComboBox->addItems({
    "astar", "jps", "bi_jps", "gbfs", "dijkstra",
    "dstar", "lpa_star", "voronoi", "dstar_lite",
    "theta_star", "lazy_theta_star", "s_theta_star",
    "hybrid_astar", "lazy"
  });

  ui_->localComboBox->addItems({
    "dwa", "pid", "apf", "rpp", "lqr", "mpc", "static"
  });

  connect(ui_->applyButton, SIGNAL(clicked()), this, SLOT(applyClicked()));
  connect(ui_->restartButton, SIGNAL(clicked()), this, SLOT(restartClicked()));
}

PlannerSelectorPanel::~PlannerSelectorPanel() {
  delete ui_;
}

void PlannerSelectorPanel::applyClicked() {
  std_msgs::String global_msg, local_msg;
  global_msg.data = ui_->globalComboBox->currentText().toStdString();
  local_msg.data = ui_->localComboBox->currentText().toStdString();
  global_pub_.publish(global_msg);
  local_pub_.publish(local_msg);
  ROS_INFO_STREAM("[PlannerPanel] Published planners: " << global_msg.data << ", " << local_msg.data);
}

void PlannerSelectorPanel::restartClicked() {
  std_srvs::Trigger srv;
  if (restart_client_.call(srv)) {
    ROS_INFO_STREAM("Restarted: " << srv.response.message);
  } else {
    ROS_WARN("Failed to call /restart_move_base service");
  }
}

}  // namespace interactive_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(interactive_tools::PlannerSelectorPanel, rviz::Panel)
