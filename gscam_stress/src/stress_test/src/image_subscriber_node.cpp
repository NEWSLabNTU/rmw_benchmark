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

    std::string topic_name = this->get_parameter("topic_name").as_string();
    double stats_interval = this->get_parameter("show_stats_interval").as_double();

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_name.c_str());

    // Create subscription with QoS for large messages
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

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
