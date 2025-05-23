#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "custom_msgs/msg/abs_result.hpp"

#define PKG_PATH "/ros2_ws/px4/src/px2/"

#include "yolo_inference.h"
#include "midas_inference.h"

namespace onnx_inference
{
class OnnxInferenceNode : public rclcpp::Node
{
public:
  explicit OnnxInferenceNode(const rclcpp::NodeOptions & options);
private:
  // Subscriber
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  
  std::string pkg_path = PKG_PATH; //ament_index_cpp::get_package_share_directory("px2");
  int decided_z = 90;
  int image_h;
  int image_w;
  cv::Mat latest_image_;
  bool image_ready_ = false;
  std::list<float> subscriber_list;
  void callbackInference();
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  double computeMedian(cv::Mat& img);
  std::list<float> SearchMedian(std::vector<Result> resultVector, cv::Mat& depth_resize, cv::Mat& yolo_result);
  void publishState(const custom_msgs::msg::AbsResult & message);
};

}  // namespace onnx_inference
#endif  // ONN_INFEREMNCE_HPP_
