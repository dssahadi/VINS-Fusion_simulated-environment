#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <algorithm>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class StereoImagePublisher : public rclcpp::Node
{
public:
  StereoImagePublisher() : Node("stereo_image_publisher")
  {
    declare_parameter<std::string>("left_image_dir", "");
    declare_parameter<std::string>("right_image_dir", "");
    declare_parameter<int>("fps", 20);

    left_dir_  = get_parameter("left_image_dir").as_string();
    right_dir_ = get_parameter("right_image_dir").as_string();
    fps_       = get_parameter("fps").as_int();

    if (left_dir_.empty() || right_dir_.empty()) {
      RCLCPP_FATAL(get_logger(), "left_image_dir or right_image_dir is empty");
      rclcpp::shutdown();
    }

    left_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/left/image_raw", 10);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/right/image_raw", 10);

    for (auto &p : fs::directory_iterator(left_dir_))
      left_images_.push_back(p.path().string());

    for (auto &p : fs::directory_iterator(right_dir_))
      right_images_.push_back(p.path().string());

    std::sort(left_images_.begin(), left_images_.end());
    std::sort(right_images_.begin(), right_images_.end());

    if (left_images_.size() != right_images_.size()) {
      RCLCPP_WARN(get_logger(),
        "Left (%ld) and Right (%ld) image counts differ",
        left_images_.size(), right_images_.size());
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / fps_),
      std::bind(&StereoImagePublisher::publish_images, this)
    );

    RCLCPP_INFO(get_logger(),
      "Publishing %ld stereo pairs at %d FPS",
      std::min(left_images_.size(), right_images_.size()), fps_);
  }

private:
  void publish_images()
  {
    if (idx_ >= left_images_.size() || idx_ >= right_images_.size())
      return;

    cv::Mat left_img  = cv::imread(left_images_[idx_],  cv::IMREAD_GRAYSCALE);
    cv::Mat right_img = cv::imread(right_images_[idx_], cv::IMREAD_GRAYSCALE);

    if (left_img.empty() || right_img.empty()) {
      RCLCPP_WARN(get_logger(), "Could not read stereo images");
      idx_++;
      return;
    }

    auto timestamp = now();

    auto left_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      "mono8",
      left_img
    ).toImageMsg();

    auto right_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      "mono8",
      right_img
    ).toImageMsg();

    left_msg->header.stamp  = timestamp;
    right_msg->header.stamp = timestamp;

    left_msg->header.frame_id  = "left_camera";
    right_msg->header.frame_id = "right_camera";

    left_pub_->publish(*left_msg);
    right_pub_->publish(*right_msg);

    idx_++;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> left_images_;
  std::vector<std::string> right_images_;
  size_t idx_ = 0;

  std::string left_dir_;
  std::string right_dir_;
  int fps_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoImagePublisher>());
  rclcpp::shutdown();
  return 0;
}

