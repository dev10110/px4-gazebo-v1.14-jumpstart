#include "teleop_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>
#include <QVBoxLayout>
#include <stdio.h>

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace dasc_robot_gui {

using std::placeholders::_1;

// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TeleopPanel::TeleopPanel(QWidget *parent) : rviz_common::Panel(parent) {
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout *topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Robot Namespace:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  // Next layout the grid of EKF and Target Setpoints
  QHBoxLayout *pos_layout = new QHBoxLayout;
  QGridLayout *grid_layout = new QGridLayout;
  grid_layout->setHorizontalSpacing(3);
  pos_layout->addItem(grid_layout);

  // header row
  // add(*Widget, row, col, rowspan, colspan);
  grid_layout->addWidget(new QLabel("[FRD]"), 0, 0);
  grid_layout->addWidget(new QLabel("x [m]"), 0, 1);
  grid_layout->addWidget(new QLabel("y [m]"), 0, 2);
  grid_layout->addWidget(new QLabel("z [m]"), 0, 3);
  grid_layout->addWidget(new QLabel("yaw [°]"), 0, 4);

  // visual inertial odom row
  grid_layout->addWidget(new QLabel("Setpoint:"), 1, 0);
  setpoint_x = new QLineEdit;
  setpoint_y = new QLineEdit;
  setpoint_z = new QLineEdit;
  setpoint_yaw = new QLineEdit;
  setpoint_pub = new QCheckBox("publish");
  setpoint_pub->setChecked(false);
  grid_layout->addWidget(setpoint_x, 1, 1);
  grid_layout->addWidget(setpoint_y, 1, 2);
  grid_layout->addWidget(setpoint_z, 1, 3);
  grid_layout->addWidget(setpoint_yaw, 1, 4);
  grid_layout->addWidget(setpoint_pub, 1, 5);

  // setpoint display
  setpoint_x_disp = new QLabel;
  setpoint_y_disp = new QLabel;
  setpoint_z_disp = new QLabel;
  setpoint_yaw_disp = new QLabel;
  for (auto s :
       {setpoint_x_disp, setpoint_y_disp, setpoint_z_disp, setpoint_yaw_disp}) {
    s->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }

  grid_layout->addWidget(new QLabel("Setpoint:"), 2, 0);
  grid_layout->addWidget(setpoint_x_disp, 2, 1);
  grid_layout->addWidget(setpoint_y_disp, 2, 2);
  grid_layout->addWidget(setpoint_z_disp, 2, 3);
  grid_layout->addWidget(setpoint_yaw_disp, 2, 4);

  // EKF row
  grid_layout->addWidget(new QLabel("EKF:"), 3, 0);
  ekf_x = new QLabel("NaN");
  ekf_y = new QLabel("NaN");
  ekf_z = new QLabel("NaN");
  ekf_yaw = new QLabel("NaN");
  ekf_valid = new QLabel("INVALID");
  for (auto e : {ekf_x, ekf_y, ekf_z, ekf_yaw, ekf_valid}) {
    e->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }
  grid_layout->addWidget(ekf_x, 3, 1);
  grid_layout->addWidget(ekf_y, 3, 2);
  grid_layout->addWidget(ekf_z, 3, 3);
  grid_layout->addWidget(ekf_yaw, 3, 4);
  grid_layout->addWidget(ekf_valid, 3, 5);

  // mocap row
  grid_layout->addWidget(new QLabel("Mocap:"), 4, 0);
  mocap_x = new QLabel("NaN");
  mocap_y = new QLabel("NaN");
  mocap_z = new QLabel("NaN");
  mocap_yaw = new QLabel("NaN");
  mocap_valid = new QLabel("Invalid");
  for (auto m : {mocap_x, mocap_y, mocap_z, mocap_yaw, mocap_valid}) {
    m->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }
  grid_layout->addWidget(mocap_x, 4, 1);
  grid_layout->addWidget(mocap_y, 4, 2);
  grid_layout->addWidget(mocap_z, 4, 3);
  grid_layout->addWidget(mocap_yaw, 4, 4);
  grid_layout->addWidget(mocap_valid, 4, 5);

  // visual inertial odom row
  grid_layout->addWidget(new QLabel("VIO:"), 5, 0);

  // params row
  QHBoxLayout *param_layout = new QHBoxLayout;
  param_name_ = new QLineEdit;
  param_name_ -> setPlaceholderText("param name");
  param_name_->setMaxLength(16);
  param_get_label_ = new QLabel("[?]");
  param_get_button_ = new QPushButton("Get");
  param_set_ = new QLineEdit;
  param_set_->setPlaceholderText("new value");
  param_set_button_ = new QPushButton("Set");
  param_layout->addWidget(new QLabel("Param:"));
  param_layout->addWidget(param_name_);
  param_layout->addWidget(param_get_button_);
  param_layout->addWidget(param_get_label_);
  param_layout->addWidget(param_set_);
  param_layout->addWidget(param_set_button_);

  // create the arm disarm buttons
  QHBoxLayout *button_layout = new QHBoxLayout;
  status_label_ = new QLabel("status: UNKNOWN");
  arm_button_ = new QPushButton("Arm", this);
  offboard_button_ = new QPushButton("Offboard", this);
  land_button_ = new QPushButton("Land", this);
  disarm_button_ = new QPushButton("Disarm", this);
  arm_button_->setDisabled(true);
  offboard_button_->setDisabled(true);
  land_button_->setDisabled(true);
  disarm_button_->setDisabled(false);
  button_layout->addWidget(status_label_);
  button_layout->addWidget(arm_button_);
  button_layout->addWidget(offboard_button_);
  button_layout->addWidget(land_button_);
  button_layout->addWidget(disarm_button_);

  // Lay out the topic field above the control widget.
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(pos_layout);
  layout->addLayout(param_layout);
  layout->addLayout(button_layout);
  setLayout(layout);

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  //
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TeleopPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer *output_timer = new QTimer(this);
  setpoint_pub_timer_ = new QTimer(this);

  // Next we make signal/slot connections.
  connect(output_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateTopic()));

  connect(arm_button_, &QPushButton::clicked, this, [this]() {
    this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_ARMED);
  });
  connect(offboard_button_, &QPushButton::clicked, this, [this]() {
    this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_OFFBOARD);
  });
  connect(land_button_, &QPushButton::clicked, this, [this]() {
    this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_LAND);
    setpoint_pub->setChecked(false);
  });
  connect(disarm_button_, &QPushButton::clicked, this, [this]() {
    this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_DISARMED);
    setpoint_pub->setChecked(false);
  });
  connect(param_get_button_, &QPushButton::clicked, this,
          [this]() { this->parameter_req(false); });
  connect(param_set_button_, &QPushButton::clicked, this,
          [this]() { this->parameter_req(true); });

  // connect the setpoint publisher
  connect(setpoint_pub, &QCheckBox::stateChanged, this, [this]() {
    if (this->setpoint_pub->isChecked()) {
      this->setpoint_pub_timer_->start(50);
    } else {
      this->setpoint_pub_timer_->stop();
    }
  });

  connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));
  connect(setpoint_pub_timer_, SIGNAL(timeout()), this,
          SLOT(setpoint_pub_timer_callback()));

  // Start the main timer.
  output_timer->start(200); // ms

  // Create the node
  node_ = std::make_shared<rclcpp::Node>("dasc_robot_gui_node");
}

void TeleopPanel::parameter_req(bool set) {

  std::string param_name = param_name_->text().toStdString(); // get the text
  if (param_name == "") {
    return;
  }
  if (param_name.length() > 16) {
    // param_name is too long
    param_get_label_->setText("[len(param_name) > 16]");
    return;
  }

  if (rclcpp::ok() && parameter_req_pub_ != NULL) {
    // construct the request message
    px4_msgs::msg::ParameterReq msg;

    const char *param_name_char =
        reinterpret_cast<const char *>(param_name.c_str());
    for (size_t i = 0; i < param_name.length(); i++) {
      msg.param_name[i] = param_name_char[i];
    }

    msg.set = set;
    if (set) {
      // need to populate parameter
      bool ok;
      float new_value = param_set_->text().toFloat(&ok);
      if (!ok) {
        std::cout << "Unable to get numeric value: only sending get request"
                  << std::endl;
        msg.set = false;
      } else {
        msg.value = new_value;
      }
    }

    parameter_req_pub_->publish(msg);
  }
}

void TeleopPanel::setpoint_pub_timer_callback() {

  if (rclcpp::ok() && trajectory_setpoint_pub_ != NULL) {
    // construct the setpoint
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position[0] = setpoint_x->text().toFloat();
    msg.position[1] = setpoint_y->text().toFloat();
    msg.position[2] = setpoint_z->text().toFloat();
    msg.yaw = (float)(M_PI / 180.f) * setpoint_yaw->text().toFloat();
    for (std::size_t i = 0; i < 3; i++) {
      msg.velocity[i] = 0;
      msg.acceleration[i] = 0;
      msg.jerk[i] = 0;
    }
    msg.yawspeed = 0;
    trajectory_setpoint_pub_->publish(msg);
  }
}

void TeleopPanel::timer_callback() {
  reset();
  rclcpp::spin_some(node_);
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void TeleopPanel::updateTopic() { setTopic(output_topic_editor_->text()); }

// Set the topic name we are publishing to.
void TeleopPanel::setTopic(const QString &new_topic) {
  // Only take action if the name has changed.
  if (new_topic != output_topic_) {
    output_topic_ = new_topic;

    // if the pub/sub exists, reset it first
    if (velocity_publisher_ != NULL) {
      velocity_publisher_.reset();
    }
    if (commander_set_state_pub_ != NULL) {
      commander_set_state_pub_.reset();
    }
    if (trajectory_setpoint_pub_ != NULL) {
      trajectory_setpoint_pub_.reset();
    }
    if (trajectory_setpoint_sub_ != NULL) {
      trajectory_setpoint_sub_.reset();
    }
    if (vehicle_local_pos_sub_ != NULL) {
      vehicle_local_pos_sub_.reset();
    }
    if (commander_status_sub_ != NULL) {
      commander_status_sub_.reset();
    }
    if (vehicle_visual_odometry_sub_ != NULL) {
      vehicle_visual_odometry_sub_.reset();
    }
    if (parameter_req_pub_ != NULL) {
      parameter_req_pub_.reset();
    }
    if (parameter_res_sub_ != NULL) {
      parameter_res_sub_.reset();
    }

    reset();

    // If the topic is the empty string, don't publish anything.
    if (output_topic_ != "") {

      // create publishers
      velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
          output_topic_.toStdString(), 1);

      commander_set_state_pub_ =
          node_->create_publisher<px4_msgs::msg::CommanderSetState>(
              output_topic_.toStdString() + "/fmu/in/commander_set_state", 1);

      trajectory_setpoint_pub_ =
          node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
              output_topic_.toStdString() + "/fmu/in/trajectory_setpoint", 1);

      parameter_req_pub_ = node_->create_publisher<px4_msgs::msg::ParameterReq>(
          output_topic_.toStdString() + "/fmu/in/parameter_req", 1);

      // create subscribers
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                             qos_profile);
      auto new_topic =
          output_topic_.toStdString() + "/fmu/out/vehicle_local_position";
      std::cout << "Subscribing to: " << new_topic << std::endl;
      vehicle_local_pos_sub_ =
          node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
              new_topic, qos,
              std::bind(&TeleopPanel::vehicle_local_pos_cb, this, _1));

      new_topic = output_topic_.toStdString() + "/fmu/out/commander_status";
      std::cout << "Subscribing to: " << new_topic << std::endl;
      commander_status_sub_ =
          node_->create_subscription<px4_msgs::msg::CommanderStatus>(
              new_topic, qos,
              std::bind(&TeleopPanel::commander_status_cb, this, _1));

      new_topic = output_topic_.toStdString() + "/fmu/in/trajectory_setpoint";
      std::cout << "Subscribing to: " << new_topic << std::endl;
      trajectory_setpoint_sub_ =
          node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
              new_topic, qos,
              std::bind(&TeleopPanel::trajectory_setpoint_cb, this, _1));

      new_topic =
          output_topic_.toStdString() + "/fmu/in/vehicle_visual_odometry";
      std::cout << "Subscribing to: " << new_topic << std::endl;
      vehicle_visual_odometry_sub_ =
          node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
              new_topic, qos,
              std::bind(&TeleopPanel::vehicle_visual_odometry_cb, this, _1));

      new_topic = output_topic_.toStdString() + "/fmu/out/parameter_res";
      std::cout << "Subscribing to: " << new_topic << std::endl;
      parameter_res_sub_ =
          node_->create_subscription<px4_msgs::msg::ParameterRes>(
              new_topic, qos,
              std::bind(&TeleopPanel::parameter_res_cb, this, _1));
    }

    // rviz_common::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz_common::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }
}

void TeleopPanel::parameter_res_cb(
    const px4_msgs::msg::ParameterRes::SharedPtr msg) const {

  if (msg->is_int32) {
    param_get_label_->setText(
        QString("[int] %1").arg(msg->value, 5, 'f', 0, ' '));
  } else {
    param_get_label_->setText(
        QString("[float] %1").arg(msg->value, 5, 'f', 3, ' '));
  }
}

void TeleopPanel::vehicle_visual_odometry_cb(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const {

  // check that the frame is NED
  if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
    for (auto m : {mocap_x, mocap_y, mocap_z, mocap_yaw}) {
      m->setText("?");
    }
    mocap_valid->setText("Frame NOT NED");
    mocap_valid->setStyleSheet("QLabel { color : red; }");
  }

  mocap_x->setText(QString("%1").arg(msg->position[0], 5, 'f', 3, ' '));
  mocap_y->setText(QString("%1").arg(msg->position[1], 5, 'f', 3, ' '));
  mocap_z->setText(QString("%1").arg(msg->position[2], 5, 'f', 3, ' '));

  tf2::Quaternion q(msg->q[3], msg->q[0], msg->q[1], msg->q[2]);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  mocap_yaw->setText(
      QString("%1").arg(float(180.0 / M_PI * yaw), 5, 'f', 1, ' '));

  mocap_valid->setText("Valid");
  mocap_valid->setStyleSheet("QLabel { color : green; }");
}

void TeleopPanel::trajectory_setpoint_cb(
    const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) const {

  // update the setpoint display
  setpoint_x_disp->setText(QString("%1").arg(msg->position[0], 5, 'f', 3, ' '));
  setpoint_y_disp->setText(QString("%1").arg(msg->position[1], 5, 'f', 3, ' '));
  setpoint_z_disp->setText(QString("%1").arg(msg->position[2], 5, 'f', 3, ' '));
  setpoint_yaw_disp->setText(
      QString("%1").arg(float(180.0 / M_PI) * msg->yaw, 5, 'f', 1, ' '));
}

void TeleopPanel::commander_status_cb(
    const px4_msgs::msg::CommanderStatus::SharedPtr msg) const {

  // ARM
  arm_button_->setDisabled(msg->state !=
                           px4_msgs::msg::CommanderStatus::STATE_DISARMED);

  // Offboard
  offboard_button_->setDisabled(msg->state !=
                                px4_msgs::msg::CommanderStatus::STATE_ARMED);

  // LAND
  land_button_->setDisabled(msg->state !=
                            px4_msgs::msg::CommanderStatus::STATE_OFFBOARD);

  // DISARM
  disarm_button_->setDisabled(msg->state ==
                              px4_msgs::msg::CommanderStatus::STATE_DISARMED);

  switch (msg->state) {

  case px4_msgs::msg::CommanderStatus::STATE_DISARMED:
    status_label_->setText("state: DISARMED");
    return;
  case px4_msgs::msg::CommanderStatus::STATE_ARMED:
    status_label_->setText("state: ARMED");
    return;
  case px4_msgs::msg::CommanderStatus::STATE_OFFBOARD:
    status_label_->setText("state: OFFBOARD");
    return;
  case px4_msgs::msg::CommanderStatus::STATE_LAND:
    status_label_->setText("state: LAND");
    return;
  default:
    return;
  }
}

void TeleopPanel::reset() {

  for (auto l :
       {setpoint_x_disp, setpoint_y_disp, setpoint_z_disp, setpoint_yaw_disp}) {
    l->setText("?");
  }
  for (auto l : {ekf_x, ekf_y, ekf_z, ekf_yaw}) {
    l->setText("?");
  }
  ekf_valid->setText("INVALID");
  ekf_valid->setStyleSheet("QLabel { color : red; }");

  for (auto l : {mocap_x, mocap_y, mocap_z, mocap_yaw}) {
    l->setText("?");
  }
  mocap_valid->setText("NO MSGS");
  mocap_valid->setStyleSheet("QLabel {color : red; }");

}

void TeleopPanel::vehicle_local_pos_cb(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const {

  if (msg->xy_valid && msg->z_valid && msg->v_xy_valid && msg->v_z_valid) {
    ekf_valid->setText("VALID");
    ekf_valid->setStyleSheet("QLabel { color : green; }");
  } else {
    ekf_valid->setText("INVALID");
    ekf_valid->setStyleSheet("QLabel { color : red; }");
  }

  ekf_x->setText(QString("%1").arg(msg->x, 5, 'f', 3, ' '));
  ekf_y->setText(QString("%1").arg(msg->y, 5, 'f', 3, ' '));
  ekf_z->setText(QString("%1").arg(msg->z, 5, 'f', 3, ' '));
  ekf_yaw->setText(
      QString("%1").arg(msg->heading * float(180.0f / M_PI), 5, 'f', 1, ' '));
}

void TeleopPanel::commander_set_state(uint8_t new_state) {
  if (rclcpp::ok() && commander_set_state_pub_ != NULL) {
    px4_msgs::msg::CommanderSetState msg;
    msg.new_state = new_state;
    commander_set_state_pub_->publish(msg);
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    output_topic_editor_->setText(topic);
    updateTopic();
  }
}

} // end namespace dasc_robot_gui

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
PLUGINLIB_EXPORT_CLASS(dasc_robot_gui::TeleopPanel, rviz_common::Panel)
// END_TUTORIAL
