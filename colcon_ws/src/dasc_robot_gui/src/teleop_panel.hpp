#ifndef TELEOP_PANEL_HPP_
#define TELEOP_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "px4_msgs/msg/commander_set_state.hpp"
#include "px4_msgs/msg/commander_status.hpp"
#include "px4_msgs/msg/parameter_req.hpp"
#include "px4_msgs/msg/parameter_res.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rviz_common/panel.hpp"
#endif
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>
#include <QDoubleSpinBox>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace dasc_robot_gui {

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz_common::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz_common::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class TeleopPanel : public rviz_common::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT

public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit TeleopPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz_common::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

  // Next come a couple of public Qt slots.

public Q_SLOTS:

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic(const QString &topic);

  // Here we declare some internal slots.

protected Q_SLOTS:
  void commander_set_state(uint8_t new_state);

  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateTopic();

  void timer_callback();
  void setpoint_pub_timer_callback();
  void parameter_req(bool set);

  void trajectory_setpoint_cb(
      const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) const;

  void vehicle_local_pos_cb(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

  void commander_status_cb(
      const px4_msgs::msg::CommanderStatus::SharedPtr msg) ;

  void vehicle_visual_odometry_cb(
      const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const;

  void parameter_res_cb(const px4_msgs::msg::ParameterRes::SharedPtr msg) const;

  void reset();

  // Then we finish up with protected member variables.

protected:
  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit *output_topic_editor_;

  // Setpoint
  QDoubleSpinBox *setpoint_x, *setpoint_y, *setpoint_z, *setpoint_yaw;
  QLabel *setpoint_x_disp, *setpoint_y_disp, *setpoint_z_disp,
      *setpoint_yaw_disp;
  QCheckBox *setpoint_pub;
  QTimer *setpoint_pub_timer_;

  // EKF:
  QLabel *ekf_x, *ekf_y, *ekf_z, *ekf_yaw, *ekf_valid;

  // Mocap:
  QLabel *mocap_x, *mocap_y, *mocap_z, *mocap_yaw, *mocap_valid;

  // Status:
  QLabel *status_label_;
  QPushButton *arm_button_, *offboard_button_, *land_button_, *disarm_button_;

  // Param:
  QLineEdit *param_name_, *param_set_;
  QLabel *param_get_label_;
  QPushButton *param_get_button_, *param_set_button_;

  // The current name of the output topic.
  QString output_topic_;

  // The ROS node and publisher for the command velocity.
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<px4_msgs::msg::CommanderSetState>::SharedPtr
      commander_set_state_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::ParameterReq>::SharedPtr parameter_req_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      vehicle_local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::CommanderStatus>::SharedPtr
      commander_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      vehicle_visual_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::ParameterRes>::SharedPtr
      parameter_res_sub_;
	
   uint64_t last_timestamp_commander_status_ = 0;
};

} // end namespace dasc_robot_gui

#endif // TELEOP_PANEL_HPP_
