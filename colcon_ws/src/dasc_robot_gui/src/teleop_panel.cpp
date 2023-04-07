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
//#include <iostream>
//#include <iomanip>

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
  grid_layout->addWidget(setpoint_x, 1, 1);
  grid_layout->addWidget(setpoint_y, 1, 2);
  grid_layout->addWidget(setpoint_z, 1, 3);
  grid_layout->addWidget(setpoint_yaw, 1, 4);
  grid_layout->addWidget(setpoint_pub, 1, 5);

  // EKF row
  grid_layout->addWidget(new QLabel("EKF:"), 2, 0);
  ekf_x = new QLabel("NaN");
  ekf_y = new QLabel("NaN");
  ekf_z = new QLabel("NaN");
  ekf_yaw = new QLabel("NaN");
  ekf_valid = new QLabel("INVALID");
  ekf_x->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  ekf_y->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  ekf_z->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  ekf_yaw->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  ekf_valid->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  grid_layout->addWidget(ekf_x, 2, 1);
  grid_layout->addWidget(ekf_y, 2, 2);
  grid_layout->addWidget(ekf_z, 2, 3);
  grid_layout->addWidget(ekf_yaw, 2, 4);
  grid_layout->addWidget(ekf_valid, 2, 5);

  // mocap odom row
  grid_layout->addWidget(new QLabel("Mocap:"), 3, 0);

  // visual inertial odom row
  grid_layout->addWidget(new QLabel("VIO:"), 4, 0);


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
  setpoint_pub_timer = new QTimer(this);

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
  });
  connect(disarm_button_, &QPushButton::clicked, this, [this]() {
    this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_DISARMED);
  });
 
  // connect the setpoint publisher 
  connect(setpoint_pub, &QCheckBox::stateChanged, this, [this]() {
		  if (this->setpoint_pub.isChecked()) {
		  	this->setpoint_pub_timer_.start(50);
			} else {
			this->setpoint_pub_timer_.stop();
			}
  });

  connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));
  connect(setpoint_pub_timer, SIGNAL(timeout()), this, SLOT(setpoint_pub_timer_callback()));

  // Start the main timer.
  output_timer->start(200); // ms

  // Create the node
  node_ = std::make_shared<rclcpp::Node>("dasc_robot_gui_node");
}

void TeleopPanel::setpoint_pub_timer_callback() {




        // TODO: continue here
	if (rclcpp.isOK() && topic != NULL){

		// construct the setpoint
		px4_msgs::msg::TrajectorySetpoint::SharedPtr msg;
		msg->position[0] = setpoint_x.text().toFloat();
		msg->position[1] = setpoint_y.text().toFloat();
		msg->position[2] = setpoint_z.text().toFloat();
		msg->yaw = setpoint_yaw.text().toFloat();
		for (std::size_t i=0; i<3; i++){
			msg->velocity[i] = 0;
			msg->accleration[i] = 0;
			msg->jerk[i] = 0;
		}
		msg->yaw_speed = 0;



	}

}

void TeleopPanel::timer_callback() { rclcpp::spin_some(node_); }

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
    // If a publisher currently exists, destroy it.
    if (velocity_publisher_ != NULL) {
      velocity_publisher_.reset();
    }
    // If a publisher currently exists, destroy it.
    if (commander_set_state_pub_ != NULL) {
      commander_set_state_pub_.reset();
    }
    // If the subscriber exists, destroy it.
    if (vehicle_local_pos_sub_ != NULL) {
      vehicle_local_pos_sub_.reset();
    }
    // If the subscriber exists, destroy it.
    if (commander_status_sub_ != NULL) {
      commander_status_sub_.reset();
    }
    // If the topic is the empty string, don't publish anything.
    if (output_topic_ != "") {
      // The call to create_publisher() says we want to publish data on the new
      // topic name.
      velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
          output_topic_.toStdString(), 1);

      commander_set_state_pub_ =
          node_->create_publisher<px4_msgs::msg::CommanderSetState>(
              output_topic_.toStdString() + "/fmu/in/commander_set_state", 1);

      reset_ekf_label();

      // Subscribe to VehicleLocalPosition
      // requires setting up qos_profile to receive new msgs
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
    status_label_->setText("state: DISABLED");
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

void TeleopPanel::reset_ekf_label() {
  ekf_valid->setText("INVALID");
  ekf_valid->setStyleSheet("QLabel { color : red; }");

  ekf_x->setText("NaN");
  ekf_y->setText("NaN");
  ekf_z->setText("NaN");
  ekf_yaw->setText("NaN");
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
