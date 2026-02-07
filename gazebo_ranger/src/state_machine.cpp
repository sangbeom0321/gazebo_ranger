#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <optional>

struct Pose2D {
  double x{0.0}, y{0.0}, yaw{0.0};
  bool valid{false};
};

static double yawFromQuat(const geometry_msgs::msg::Quaternion &q){
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

static double angDiffRad(double a, double b){
  double d = std::fmod(a - b + M_PI, 2*M_PI);
  if (d < 0) d += 2*M_PI;
  return std::fabs(d - M_PI);
}

class ModePublisherNode : public rclcpp::Node {
public:
  ModePublisherNode() : Node("mode_publisher_node"),
                        tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    // Parameters
    model_states_topic_ = declare_parameter<std::string>("model_states_topic", "/world/model_states_world");
    marker_topic_       = declare_parameter<std::string>("marker_topic", "/aruco_marker_pose_global");
    model_name_         = declare_parameter<std::string>("model_name", "ranger_mini");
    world_frame_        = declare_parameter<std::string>("world_frame", "world");
    publish_rate_hz_    = declare_parameter<double>("publish_rate_hz", 50.0);

    arrival_dist_m_     = declare_parameter<double>("arrival_distance_m", 0.10);  // 10 cm
    arrival_yaw_deg_    = declare_parameter<double>("arrival_yaw_deg", 5.0);      // 5 deg
    change_pos_m_       = declare_parameter<double>("marker_change_position_m", 0.10);
    change_yaw_deg_     = declare_parameter<double>("marker_change_yaw_deg", 5.0);

    arrival_yaw_rad_    = deg2rad(arrival_yaw_deg_);
    change_yaw_rad_     = deg2rad(change_yaw_deg_);

    // Publishers
    mod_pub_  = create_publisher<std_msgs::msg::Int32>("/Control/mod", 10);
    goal_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/goal_point", 1);

    // Subscribers
    model_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
        model_states_topic_, 10,
        std::bind(&ModePublisherNode::onModelStates, this, std::placeholders::_1));

    marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        marker_topic_, 10,
        std::bind(&ModePublisherNode::onMarkerPose, this, std::placeholders::_1));

    // Periodic publisher (for /Control/mod)
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&ModePublisherNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "ModePublisherNode started. Comparing model '%s' vs marker topic '%s', publishing to /Control/mod & /Planning/goal_point",
      model_name_.c_str(), marker_topic_.c_str());
  }

private:
  // Subscriptions & Publishers
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mod_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string model_states_topic_, marker_topic_, model_name_, world_frame_;
  double publish_rate_hz_;
  double arrival_dist_m_, arrival_yaw_deg_, change_pos_m_, change_yaw_deg_;
  double arrival_yaw_rad_, change_yaw_rad_;

  // State
  Pose2D ranger_{}, marker_{}, last_marker_{};
  bool have_last_marker_{false};

  static double deg2rad(double d){ return d * M_PI / 180.0; }

  void onModelStates(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
    // find index of model_name_
    auto it = std::find(msg->name.begin(), msg->name.end(), model_name_);
    if (it == msg->name.end()){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Model [%s] not found in ModelStates.", model_name_.c_str());
      ranger_.valid = false;
      return;
    }
    size_t idx = std::distance(msg->name.begin(), it);
    const auto &p = msg->pose[idx];

    ranger_.x = p.position.x;
    ranger_.y = p.position.y;
    ranger_.yaw = yawFromQuat(p.orientation);
    ranger_.valid = true;
  }

  void onMarkerPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    geometry_msgs::msg::PoseStamped in = *msg;
    geometry_msgs::msg::PoseStamped out;

    bool transformed = false;
    try {
      auto tf = tf_buffer_.lookupTransform(world_frame_, in.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(in, out, tf);
      transformed = true;
    } catch(const std::exception &e){
      // fallback: use as-is (frame mismatch risk)
      out = in;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF transform %s -> %s failed (%s). Using raw pose; ensure frames are consistent.",
        in.header.frame_id.c_str(), world_frame_.c_str(), e.what());
    }

    (void)transformed; // unused flag

    marker_.x = out.pose.position.x;
    marker_.y = out.pose.position.y;
    marker_.yaw = yawFromQuat(out.pose.orientation);
    marker_.valid = true;

    // ✅ Publish current goal point = ArUco marker (x, y, yaw)
    std_msgs::msg::Float64MultiArray goal_point_msg;
    goal_point_msg.data = {marker_.x, marker_.y, marker_.yaw};
    goal_pub_->publish(goal_point_msg);

    // track significant marker changes (for mode logic)
    if (have_last_marker_){
      double dx = marker_.x - last_marker_.x;
      double dy = marker_.y - last_marker_.y;
      double pos_delta = std::hypot(dx, dy);
      double yaw_delta = angDiffRad(marker_.yaw, last_marker_.yaw);

      if (pos_delta > change_pos_m_ || yaw_delta > change_yaw_rad_){
        RCLCPP_INFO(get_logger(),
          "Marker changed: Δpos=%.3f m, Δyaw=%.1f deg (thresholds: %.2f m, %.1f deg)",
          pos_delta, yaw_delta * 180.0 / M_PI, change_pos_m_, change_yaw_deg_);
      }
    }
    last_marker_ = marker_;
    have_last_marker_ = true;
  }

  void onTimer(){
    int32_t mode = 2; // default: en route
    bool arrived = false;
    bool marker_changed_recently = false;

    if (ranger_.valid && marker_.valid){
      double dx = marker_.x - ranger_.x;
      double dy = marker_.y - ranger_.y;
      double dist = std::hypot(dx, dy);
      double yaw_delta = angDiffRad(ranger_.yaw, marker_.yaw);

      arrived = (dist <= arrival_dist_m_) && (yaw_delta <= arrival_yaw_rad_);

      // compare current marker to last_marker_
      if (have_last_marker_){
        double mdx = marker_.x - last_marker_.x;
        double mdy = marker_.y - last_marker_.y;
        double mpos = std::hypot(mdx, mdy);
        double myaw = angDiffRad(marker_.yaw, last_marker_.yaw);
        marker_changed_recently = (mpos > change_pos_m_) || (myaw > change_yaw_rad_);
      }

      if (arrived && !marker_changed_recently){
        mode = 3;
      } else {
        mode = 2;
      }

      RCLCPP_DEBUG(get_logger(),
        "ranger(%.3f,%.3f,%.1f°)  marker(%.3f,%.3f,%.1f°)  dist=%.3f m  yawΔ=%.1f°  -> mode %d",
        ranger_.x, ranger_.y, ranger_.yaw*180.0/M_PI,
        marker_.x, marker_.y, marker_.yaw*180.0/M_PI,
        dist, yaw_delta*180.0/M_PI, mode);
    } else {
      // until both are valid, keep publishing 2
      mode = 2;
    }

    std_msgs::msg::Int32 out;
    out.data = mode;
    mod_pub_->publish(out);
  }
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
