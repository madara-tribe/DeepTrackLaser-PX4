#include "inference.h"
#include "tool.h"

#define ONNX_DEPTH_PATH "/weights/dpt_large_384.onnx"
#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define IMG_PATH "data/+30cm.jpg"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640


using namespace std::chrono_literals;

namespace onnx_inference
{
  OnnxInferenceNode::OnnxInferenceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("px2", options)
  {
    publisher_ = this->create_publisher<custom_msgs::msg::AbsResult>("inference", rclcpp::QoS{10}.transient_local()); 
    timer_ = this->create_wall_timer(
        500ms, std::bind(&OnnxInferenceNode::callbackInference, this));
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/video_stream", rclcpp::SensorDataQoS(),
      std::bind(&OnnxInferenceNode::imageCallback, this, std::placeholders::_1));
  }
  
  void OnnxInferenceNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      latest_image_= cv_bridge::toCvCopy(msg, "bgr8")->image;
      image_ready_ = true;     
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void OnnxInferenceNode::callbackInference()
  {
      if (!image_ready_ || latest_image_.empty()) {
	  RCLCPP_WARN(this->get_logger(), "Latest image is empty or already processed.");
          return;
      }
      image_ready_ = false;  // reset after processing
      RCLCPP_INFO(rclcpp::get_logger("ImageSubscriber"), "Image received: %dx%d", latest_image_.cols, latest_image_.rows);
      
      // Generate timestamped filename
      //auto now = std::chrono::system_clock::now();
      //std::time_t now_c = std::chrono::system_clock::to_time_t(now);
      //std::stringstream filename;
      //filename << "/ros2_ws/px4/images/px2_image_"
      //   << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".jpg";
      //cv::imwrite(filename.str(), latest_image_);
      
      //std::cout << "Found image: " << pkg_path + IMG_PATH << std::endl;
      cv::Mat yolo_image = latest_image_;
      cv::Mat midas_image = latest_image_; //cv::imread(pkg_path + IMG_PATH);
      bool useCUDA{false};
      MidasInference midas(pkg_path + ONNX_DEPTH_PATH, useCUDA); // Midas instance
      std::cout << "Midas and YOLO Prepare" << yolo_image.channels() << "w" <<yolo_image.cols<< "h" << yolo_image.rows << std::endl;
      YoloDetect yolo_detector(pkg_path + ONNX_YOLO_PATH); // YOLO instance
      auto start = std::chrono::high_resolution_clock::now();
      // yolo
      cv::Mat inputImage = yolo_detector.preprocess(yolo_image, YOLO_INPUT_H, YOLO_INPUT_W);
      std::vector<Ort::Value> outputTensors = yolo_detector.RunInference(inputImage);
      std::vector<Result> resultVector = yolo_detector.postprocess(yolo_image.size(), outputTensors);
      cv::Mat yolo_result = yolo_detector.drawBoundingBox(yolo_image, resultVector);
      // midas
      cv::Mat depth_resize = midas.runInference(midas_image, pkg_path);
    
      image_h = depth_resize.rows;
      image_w = depth_resize.cols;
      std::list<float> subscribe_dist = SearchMedian(resultVector, depth_resize, yolo_result);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff = end - start;
      std::cout << "Prediction took " << diff.count() << " seconds" << std::endl;
      
      custom_msgs::msg::AbsResult message;
      auto it = subscribe_dist.begin();
      message.abs_dist = *it;
      ++it;
      message.x_real_coordinate = *it;
      message.max_x = max_x;
      publishState(message); 
  }

  std::list<float> OnnxInferenceNode::SearchMedian(std::vector<Result> resultVector, cv::Mat& depth_resize, cv::Mat& yolo_result){
    std::string imgPath = IMG_PATH;
    size_t lastSlash = imgPath.find_last_of("/");
    size_t lastDot = imgPath.find_last_of(".");
    std::list<float> subscriber_list;
    std::string baseName = imgPath.substr(lastSlash + 1, lastDot - lastSlash - 1);
    std::string outputPath = "/data/" + baseName + "_result.png";
    
    for( auto result : resultVector ) {
	int x = static_cast<int>(result.x1);
        int y = static_cast<int>(result.y1);
        int w = static_cast<int>(result.x2);
        int h = static_cast<int>(result.y2);
        
        // Fix negative values
        if (x < 0) x = 0;
        if (y < 0) y = 0;

        // Check ranges inside image size
        if (x >= depth_resize.cols || y >= depth_resize.rows) {
            RCLCPP_WARN(this->get_logger(), "Invalid ROI: out of bounds (x=%d, y=%d)", x, y);
            continue;
        }

        // Adjust w, h to be inside bounds
        w = std::min(w, depth_resize.cols);
        h = std::min(h, depth_resize.rows);	
	if (x >= w || y >= h) {
            RCLCPP_WARN(this->get_logger(), "Invalid ROI: x >= w or y >= h (x=%d, w=%d, y=%d, h=%d)", x, w, y, h);
            continue;
        }
	cv::Mat cropDepth = depth_resize(cv::Range(y, h), cv::Range(x, w)).clone();
        if (cropDepth.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty crop detected at (x=%d, y=%d)", x, y);
            continue;
        }

	double relative_dist = computeMedian(cropDepth);
        //float bbox_size = calculateBboxSize(x, y, w, h);
        double x_real_coordinate = calculateRealCoordinate(x, w);
	double abs_dist = quadraticFunction(relative_dist);
	abs_dist = decided_z;

	std::cout << "Horizontal real distance: " << x_real_coordinate << "cm" << std::endl;
        std::cout << "relative_dist:" << relative_dist << "absolute dist:" << abs_dist << std::endl;

	subscriber_list.push_back(abs_dist);
	subscriber_list.push_back(x_real_coordinate);

	cv::putText(yolo_result, std::to_string(x_real_coordinate),
                        cv::Point(x, y+50), cv::FONT_ITALIC,
                        0.8, cv::Scalar(255, 255, 0), 2);
	if (!subscriber_list.empty()){
	    break;
	}
    }
    cv::imwrite(pkg_path + outputPath, yolo_result);
    return subscriber_list;
  }
  void OnnxInferenceNode::publishState(const custom_msgs::msg::AbsResult & message)
  {
     RCLCPP_INFO(this->get_logger(), "Publishing absolute_dist: %.2f, x_real_coordinate: %.2f, max_x: %.2f",
              message.abs_dist, message.x_real_coordinate, message.max_x);
     publisher_->publish(message);
  }

  double OnnxInferenceNode::computeMedian(cv::Mat& img) {
    // Flatten image pixels into vector
    std::vector<uchar> pixels(img.datastart, img.dataend);
    size_t n = pixels.size();
    if (n == 0) return -1.0f;

    auto mid_iter = pixels.begin() + n / 2;
    std::nth_element(pixels.begin(), mid_iter, pixels.end());
    double median = static_cast<float>(*mid_iter);

    if (n % 2 == 0) {
        auto lower_max = std::max_element(pixels.begin(), mid_iter);
        median = (median + static_cast<float>(*lower_max)) / 2.0f;
    }
    return median;
  }  
}  // namespace onnx_inference


int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  rclcpp::ExecutorOptions exe_options = rclcpp::ExecutorOptions();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<onnx_inference::OnnxInferenceNode>(options);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exe_options, 2);
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();

  return 0;
}
