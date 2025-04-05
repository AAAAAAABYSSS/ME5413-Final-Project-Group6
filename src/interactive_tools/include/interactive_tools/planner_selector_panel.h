#ifndef PLANNER_SELECTOR_PANEL_H
#define PLANNER_SELECTOR_PANEL_H

#include <QWidget>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <rviz/panel.h>

namespace Ui {
class PlannerSelectorPanel;
}

namespace interactive_tools {

class PlannerSelectorPanel : public rviz::Panel
{
  Q_OBJECT

public:
  PlannerSelectorPanel(QWidget* parent = nullptr);
  virtual ~PlannerSelectorPanel();

private Q_SLOTS:
  void applyClicked();
  void restartClicked();

private:
  Ui::PlannerSelectorPanel* ui_;
  ros::NodeHandle nh_;
  ros::Publisher global_pub_;
  ros::Publisher local_pub_;
  ros::ServiceClient restart_client_;
};

}  // namespace interactive_tools

#endif  // PLANNER_SELECTOR_PANEL_H
