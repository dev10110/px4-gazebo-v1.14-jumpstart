#include <cstdio>

// #include <Eigen/Eigen>
// #include <Eigen/Geometry>

#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class GazeboPX4Bridge : public rclcpp::Node {

public:
  GazeboPX4Bridge() : Node("gazebo_px4_bridge") {
    // PARAMETERS
    px4_name_ = this->declare_parameter<std::string>("px4_name", "px4_1");

    // PUBS
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    std::string px4_pub_name = px4_name_ + "/fmu/in/vehicle_visual_odometry";
    px4_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_pub_name, 10);

    // SUBS
    std::string gazebo_odom_topic_name = "/gazebo/odom";
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        gazebo_odom_topic_name, 10,
        [this](const nav_msgs::msg::Odometry::UniquePtr msg) {
          this->odom_callback(*msg);
        });

    std::string timesync_sub_name = px4_name_ + "/fmu/out/timesync";
    timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        timesync_sub_name, 10,
        [this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
          this->px4_timestamp_.store(msg->timestamp);
          this->px4_server_timestamp_.store(
              this->get_clock()->now().nanoseconds());
        });
  }

private:
  uint64_t get_current_timestamp() {
    auto delta =
        (this->get_clock()->now().nanoseconds() - this->px4_server_timestamp_) /
        1e6;
    return px4_timestamp_.load() + delta;
  }

  void odom_callback(const nav_msgs::msg::Odometry &msg) {

    // publish as visual odometry msg
    px4_msgs::msg::VehicleOdometry px4_odom_msg;
    px4_odom_msg.timestamp = get_current_timestamp();
    px4_odom_msg.timestamp_sample = px4_odom_msg.timestamp;

    px4_odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    px4_odom_msg.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    // even though we get velocity from gazebo, we choose not to publish it, to
    // better emulate vicon

    // position
    px4_odom_msg.position[0] = msg.pose.pose.position.y;
    px4_odom_msg.position[1] = msg.pose.pose.position.x;
    px4_odom_msg.position[2] = -msg.pose.pose.position.z;

    // quaternion
    auto gazebo_q = msg.pose.pose.orientation;
    tf2::Quaternion tf2_gazebo(gazebo_q.x, gazebo_q.y, gazebo_q.z, gazebo_q.w);
    auto yaw = tf2::getYaw(tf2_gazebo);
    auto yaw_ned = -yaw + 1.57;
    px4_odom_msg.q[0] = cos(yaw_ned/2.0);
    px4_odom_msg.q[1] = 0;
    px4_odom_msg.q[2] = 0;
    px4_odom_msg.q[3] = sin(yaw_ned/2.0);

    for (uint8_t i = 0; i < 3; i++) {
      px4_odom_msg.velocity[i] = NAN;
      px4_odom_msg.angular_velocity[i] = NAN;
      px4_odom_msg.position_variance[i] = 0.01;
      px4_odom_msg.orientation_variance[i] = 0.0523; // approx 3 degrees
      px4_odom_msg.velocity_variance[i] = NAN;
    }

    px4_odom_msg.reset_counter = 0;

    px4_odometry_pub_->publish(px4_odom_msg);

    // publish to tf
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = msg.header.stamp;
    tf_msg.header.frame_id = "vicon/world"; // TO SIMULATE VICON	
    tf_msg.child_frame_id = "vicon/" + px4_name_ + "/" + px4_name_;
    tf_msg.transform.translation.x = msg.pose.pose.position.x;
    tf_msg.transform.translation.y = msg.pose.pose.position.y;
    tf_msg.transform.translation.z = msg.pose.pose.position.z;
    tf_msg.transform.rotation = msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      px4_odometry_pub_;
  std::string px4_name_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  std::atomic<uint64_t> px4_timestamp_;
  std::atomic<uint64_t> px4_server_timestamp_;

}; // class GazeboPX4Bridge





int main(int argc, char ** argv)
{

  printf("starting gazebo_px4_bridge package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPX4Bridge>());
  rclcpp::shutdown();

  return 0;
}
