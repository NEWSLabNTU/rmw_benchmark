// Copyright 2025 Autoware RMW Benchmark
// Image subscriber composable node for stress testing

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <memory>

namespace stress_test
{

class ImageSubscriberNode : public rclcpp::Node
{
public:
  explicit ImageSubscriberNode(const rclcpp::NodeOptions & options)
  : Node("image_subscriber", options)
  {
    // Declare parameters
    this->declare_parameter("topic_name", "/camera/image_raw");
    this->declare_parameter("show_stats_interval", 1.0);

    // QoS parameters
    this->declare_parameter("qos_reliability", "RELIABLE");
    this->declare_parameter("qos_durability", "VOLATILE");
    this->declare_parameter("qos_history", "KEEP_LAST");
    this->declare_parameter("qos_history_depth", 10);
    this->declare_parameter("qos_deadline_ms", 0);
    this->declare_parameter("qos_liveliness", "AUTOMATIC");
    this->declare_parameter("qos_liveliness_lease_ms", 0);

    std::string topic_name = this->get_parameter("topic_name").as_string();
    double stats_interval = this->get_parameter("show_stats_interval").as_double();

    // Read QoS parameters
    std::string qos_reliability = this->get_parameter("qos_reliability").as_string();
    std::string qos_durability = this->get_parameter("qos_durability").as_string();
    std::string qos_history = this->get_parameter("qos_history").as_string();
    int qos_history_depth = this->get_parameter("qos_history_depth").as_int();
    int qos_deadline_ms = this->get_parameter("qos_deadline_ms").as_int();
    std::string qos_liveliness = this->get_parameter("qos_liveliness").as_string();
    int qos_liveliness_lease_ms = this->get_parameter("qos_liveliness_lease_ms").as_int();

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "QoS: %s/%s, History: %s(%d)",
                qos_reliability.c_str(), qos_durability.c_str(),
                qos_history.c_str(), qos_history_depth);

    // Create subscription with configured QoS
    auto qos = (qos_history == "KEEP_ALL") ?
               rclcpp::QoS(rclcpp::KeepAll()) :
               rclcpp::QoS(rclcpp::KeepLast(qos_history_depth));

    // Set reliability
    if (qos_reliability == "BEST_EFFORT") {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    // Set durability
    if (qos_durability == "TRANSIENT_LOCAL") {
      qos.transient_local();
    } else {
      qos.durability_volatile();
    }

    // Set deadline if specified
    if (qos_deadline_ms > 0) {
      qos.deadline(std::chrono::milliseconds(qos_deadline_ms));
    }

    // Set liveliness
    if (qos_liveliness == "MANUAL_BY_TOPIC") {
      qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);
    } else if (qos_liveliness == "SYSTEM_DEFAULT") {
      qos.liveliness(rclcpp::LivelinessPolicy::SystemDefault);
    } else {
      // Default to AUTOMATIC
      qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
    }

    // Set liveliness lease duration if specified
    if (qos_liveliness_lease_ms > 0) {
      qos.liveliness_lease_duration(std::chrono::milliseconds(qos_liveliness_lease_ms));
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name,
      qos,
      std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1));

    // Create timer for statistics
    stats_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(stats_interval),
      std::bind(&ImageSubscriberNode::print_statistics, this));

    start_time_ = this->now();
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    frame_count_++;
    total_bytes_ += msg->data.size();

    // Calculate latency
    auto now = this->now();
    auto stamp = rclcpp::Time(msg->header.stamp);
    auto latency = (now - stamp).seconds() * 1000.0; // Convert to ms

    total_latency_ms_ += latency;

    if (latency > max_latency_ms_) {
      max_latency_ms_ = latency;
    }
    if (frame_count_ == 1 || latency < min_latency_ms_) {
      min_latency_ms_ = latency;
    }

    // Store image properties from first frame
    if (frame_count_ == 1) {
      width_ = msg->width;
      height_ = msg->height;
      encoding_ = msg->encoding;

      RCLCPP_INFO(this->get_logger(),
                  "First frame received: %ux%u, encoding: %s, size: %zu bytes",
                  msg->width, msg->height, msg->encoding.c_str(), msg->data.size());
    }
  }

  void print_statistics()
  {
    if (frame_count_ == 0) {
      RCLCPP_WARN(this->get_logger(), "No frames received yet");
      return;
    }

    auto elapsed = (this->now() - start_time_).seconds();
    double fps = frame_count_ / elapsed;
    double avg_latency = total_latency_ms_ / frame_count_;
    double mbps = (total_bytes_ * 8.0) / (elapsed * 1024.0 * 1024.0);

    RCLCPP_INFO(this->get_logger(),
                "\n=== Image Subscriber Statistics ===\n"
                "  Resolution: %ux%u (%s)\n"
                "  Frames: %lu\n"
                "  FPS: %.2f\n"
                "  Bandwidth: %.2f Mbps\n"
                "  Latency - Avg: %.2f ms, Min: %.2f ms, Max: %.2f ms\n"
                "  Runtime: %.2f seconds",
                width_, height_, encoding_.c_str(),
                frame_count_,
                fps,
                mbps,
                avg_latency, min_latency_ms_, max_latency_ms_,
                elapsed);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  rclcpp::Time start_time_;
  uint64_t frame_count_ = 0;
  uint64_t total_bytes_ = 0;
  double total_latency_ms_ = 0.0;
  double min_latency_ms_ = 0.0;
  double max_latency_ms_ = 0.0;

  uint32_t width_ = 0;
  uint32_t height_ = 0;
  std::string encoding_;
};

}  // namespace stress_test

RCLCPP_COMPONENTS_REGISTER_NODE(stress_test::ImageSubscriberNode)
