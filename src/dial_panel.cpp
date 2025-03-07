#include <waypoints_sender_rviz/dial_panel.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include "ui_dial_panel.h"


namespace waypoints_sender_rviz
{
DialPanel::DialPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::DialUI())
{
  topic_name_ = "/drone/path";
  
  ui_->setupUi(this);
}

DialPanel::~DialPanel() = default;

void DialPanel::onInitialize()
{
  connect(ui_->dial, SIGNAL(valueChanged(int)), this , SLOT(dialValueChanged(int)));
  connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui_->executeBtn, SIGNAL(clicked()), this, SLOT(executePath()));

  ui_->line_edit->setPlaceholderText("Input topic name (Default : dial)");

  connect(ui_->line_edit,  SIGNAL(textChanged(const QString &)), this, SLOT(lineEditChanged()));

  pub_ = nh_.advertise<visualization_msgs::Marker>(topic_name_, 1);
  parentWidget()->setVisible(true);
}

void DialPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void DialPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void DialPanel::lineEditChanged()
{
  std::string old_topic_name = topic_name_;
  if(ui_->line_edit->text().isEmpty())
    topic_name_ = "/drone/path";
  else
    topic_name_ = ui_->line_edit->text().toStdString();

  ROS_INFO("You set the topic name : %s", topic_name_.c_str());

  if(old_topic_name != topic_name_)
    pub_ = nh_.advertise<visualization_msgs::Marker>(topic_name_, 1);
}

void DialPanel::dialValueChanged(int value)
{
  ui_->lcd->display(value);
  value_ = value;
  ROS_INFO("You set the value : %d", value_);
}

void DialPanel::buttonClicked()
{
  return ui_->multirobot->isChecked() ? multiDronePath() : singleDronePath();

}

void DialPanel::executePath()
{
  if(ui_->multirobot->isChecked())
    return;

  std::string prefix = (ui_->bebop7->isChecked()) ? "/bebop7" : "/bebop5";
  bool orientation =  ui_->orientationBox->isChecked();
  wp_sender_.set(prefix, goalCounter_, value_, orientation);
  
}

void DialPanel::singleDronePath()
{
  auto Eight = Eight::PathGenerator();
  Eight.setNumPoints(150);
  Eight.setPathScale(3.5, 1.5);
  auto path = Eight.generatePath(0.0);

  auto Drone = Drone::PathViewer();
  Drone.setPath(path);
  Drone.setMarkerScale(0.1);
  Drone.setMarkerColor(0.0, 1.0, 0.0, 1.0);
  Drone.setMarkerType(visualization_msgs::Marker::SPHERE_LIST);
  Drone.setMarkerLifetime(0.0);
  Drone.setMarkerFrameId("map");

  auto msg = Drone.generateMarker();
  msg.header.stamp = ros::Time::now();
  pub_.publish(msg);
  // ROS_INFO_STREAM("Publishing path to rviz. " << path.size() << " points.");
}


void DialPanel::multiDronePath()
{
  float t0 = 31.0;
  float t1 = 70.0;

  Eight::PathGenerator bebop5, bebop7;
  auto path5 = bebop5.generatePath(t0);
  auto path7 = bebop7.generatePath(t1);


  Drone::PathViewer bebop5_viewer, bebop7_viewer;
  bebop5_viewer.setPath(path5);
  bebop5_viewer.setMarkerScale(0.1);
  bebop5_viewer.setMarkerColor(1.0, 0.0, 0.0, 1.0);
  bebop5_viewer.setMarkerType(visualization_msgs::Marker::SPHERE_LIST);
  bebop5_viewer.setMarkerLifetime(0.0);
  bebop5_viewer.setMarkerFrameId("map");  

  bebop7_viewer.setPath(path7);
  bebop7_viewer.setMarkerScale(0.1);
  bebop7_viewer.setMarkerColor(0.0, 0.0, 1.0, 1.0);
  bebop7_viewer.setMarkerType(visualization_msgs::Marker::SPHERE_LIST);
  bebop7_viewer.setMarkerLifetime(0.0);
  bebop7_viewer.setMarkerFrameId("map");  

  auto msg5 = bebop5_viewer.generateMarker();
  auto msg7 = bebop7_viewer.generateMarker();
  msg5.header.stamp = ros::Time::now();
  msg7.header.stamp = ros::Time::now();
  msg5.id = 0;
  msg5.ns = "bebop5";
  msg7.id = 1;
  msg7.ns = "bebop7";
  pub_.publish(msg5);
  pub_.publish(msg7);

}

}  // namespace waypoints_sender_rviz

PLUGINLIB_EXPORT_CLASS(waypoints_sender_rviz::DialPanel, rviz::Panel )
