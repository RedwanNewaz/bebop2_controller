#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>
#include <waypoints_sender_rviz/path_generator.h>
#include <waypoints_sender_rviz/path_viewer.h>
#include <waypoints_sender_rviz/action_client.h>

namespace Ui {
class DialUI;
}

namespace waypoints_sender_rviz
{
class DialPanel: public rviz::Panel
{
  Q_OBJECT
 public:
  DialPanel(QWidget* parent = nullptr);
  ~DialPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void dialValueChanged(int value);
  void lineEditChanged();
  void buttonClicked();
  void executePath();

private:
  void singleDronePath();
  void multiDronePath();
    

protected:
  Ui::DialUI* ui_;
  int value_{0};
  std::string topic_name_{"dial"};
  

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  int goalCounter_{0};
  WaypointsActionGoal wp_sender_;
};
} // end namespace rviz_plugin_examples
