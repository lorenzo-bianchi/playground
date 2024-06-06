/**
 * Object Detector node implementation.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 4, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <string>
#include <stdexcept>

#include <object_detector/object_detector.hpp>

namespace ObjectDetector
{

/**
 * @brief Builds a new Object Detector node.
 *
 * @param opts Node options.
 *
 * @throws RuntimeError if initialization fails.
 */
ObjectDetectorNode::ObjectDetectorNode(const rclcpp::NodeOptions & node_options)
: NodeBase("object_detector", node_options, true)
{
  init_parameters();
  init_publishers();
  init_subscriptions();
  init_services();
  init_inference();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Finalizes node operation.
 */
ObjectDetectorNode::~ObjectDetectorNode()
{
  if (running_.load(std::memory_order_acquire)) {
    // Join worker thread
    running_.store(false, std::memory_order_release);
    sem_post(&sem1_);
    sem_post(&sem2_);
    worker_.join();

    // Shut down camera subscriber
    camera_sub_->shutdown();
    camera_sub_.reset();

    // Destroy semaphores
    sem_destroy(&sem1_);
    sem_destroy(&sem2_);
  }
  stream_pub_.reset();
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void ObjectDetectorNode::init_inference()
{
  cv::Size size = cv::Size(image_dims_[0], image_dims_[1]);
  detector = Inference(onnx_path_, size, use_gpu_, &model_score_threshold_, &model_NMS_threshold_);
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void ObjectDetectorNode::init_subscriptions()
{
  if (autostart_) {
    // Initialize semaphores
    sem_init(&sem1_, 0, 1);
    sem_init(&sem2_, 0, 0);

    // Spawn worker thread
    running_.store(true, std::memory_order_release);
    worker_ = std::thread(
      &ObjectDetectorNode::worker_thread_routine,
      this);
    if (worker_cpu_ != -1) {
      cpu_set_t worker_cpu_set;
      CPU_ZERO(&worker_cpu_set);
      CPU_SET(worker_cpu_, &worker_cpu_set);
      if (pthread_setaffinity_np(
          worker_.native_handle(),
          sizeof(cpu_set_t),
          &worker_cpu_set))
      {
        char err_msg_buf[100] = {};
        char * err_msg = strerror_r(errno, err_msg_buf, 100);
        throw std::runtime_error(
                "ObjectDetectorNode::init_subscriptions: Failed to configure worker thread: " +
                std::string(err_msg));
      }
    }

    // Subscribe to image topic
    camera_sub_ = std::make_shared<image_transport::CameraSubscriber>(
      image_transport::create_camera_subscription(
        this,
        input_topic_,
        std::bind(
          &ObjectDetectorNode::camera_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2),
        transport_,
        best_effort_sub_qos_ ?
        DUAQoS::Visualization::get_image_qos(image_sub_depth_).get_rmw_qos_profile() :
        DUAQoS::get_image_qos(image_sub_depth_).get_rmw_qos_profile()));
  }
}

/**
 * @brief Routine to initialize topic publishers.
 */
void ObjectDetectorNode::init_publishers()
{
  // Targets data
  target_array_pub_ = this->create_publisher<TargetArray>(
    output_topic_,
    DUAQoS::get_datum_qos());

  // Visual targets data
  visual_targets_pub_ = this->create_publisher<VisualTargets>(
    visual_data_topic_,
    DUAQoS::get_datum_qos());

  // Targets detection stream
  stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    stream_topic_,
    DUAQoS::Visualization::get_image_qos(image_sub_depth_).get_rmw_qos_profile());
}

/**
 * @brief Routine to initialize service servers.
 */
void ObjectDetectorNode::init_services()
{
  // Enable
  enable_server_ = this->create_service<SetBool>(
    "~/enable",
    std::bind(
      &ObjectDetectorNode::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

/**
 * @brief Worker routine.
 */
void ObjectDetectorNode::worker_thread_routine()
{
  while (true) {
    // Get new data
    std_msgs::msg::Header header_;
    cv::Mat image_{};
    sem_wait(&sem2_);
    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    image_ = new_frame_.clone();
    header_ = last_header_;
    sem_post(&sem1_);

    // Detect targets
    std::vector<Detection> output = detector.runInference(image_);
    int detections = output.size();

    // print output
    for (int i = 0; i < detections; ++i)
    {
      Detection detection = output[i];
      RCLCPP_INFO(this->get_logger(), "Detected %s at (%d, %d, %d, %d) with confidence %f",
                  detection.className.c_str(), detection.box.x, detection.box.y,
                  detection.box.width, detection.box.height, detection.confidence);
    }

    // Return if no target is detected
    if (detections == 0) {continue;}

    for (int i = 0; i < detections; ++i)
    {
      Detection detection = output[i];

      cv::Rect box = detection.box;
      cv::Scalar color = detection.color;

      // Detection box
      cv::rectangle(image_, box, color, 2);

      // Detection box text
      std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
      cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
      cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

      cv::rectangle(image_, textBox, color, cv::FILLED);
      cv::putText(image_, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
    }

    camera_frame_ = image_; // Doesn't copy image data, but sets data type...

    // Create processed image message
    Image::SharedPtr processed_image_msg = frame_to_msg(camera_frame_);
    processed_image_msg->set__header(header_);

    // Publish processed image
    stream_pub_->publish(processed_image_msg);
  }

  RCLCPP_WARN(this->get_logger(), "Object Detector DEACTIVATED");
}

} // namespace ObjectDetector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ObjectDetector::ObjectDetectorNode)