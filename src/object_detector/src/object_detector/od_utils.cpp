/**
 * Object Detector node auxiliary functions.
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

#include <object_detector/object_detector.hpp>

namespace ObjectDetector
{

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr ObjectDetectorNode::frame_to_msg(cv::Mat & frame)
{
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->set__step(frame.cols * frame.elemSize());

  // Check data endianness
  int num = 1;
  ros_image->set__is_bigendian(!(*(char *)&num == 1));

  // Copy frame data (this avoids the obsolete cv_bridge)
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

/*  */
Inference::Inference(std::string& onnx_model_path,
                     cv::Size model_input_shape,
                     bool run_with_cuda,
                     double* score_threshold,
                     double* nms_threshold)
{
  this->model_path = onnx_model_path;
  this->model_shape = model_input_shape;
  this->cuda_enabled = run_with_cuda;
  this->score_threshold = score_threshold;
  this->nms_threshold = nms_threshold;

  // TODO: remove
  std::random_device rd;
  gen = std::mt19937(rd());
  dis = std::uniform_int_distribution<int>(100, 255);

  load_onnx_network();
}

std::vector<Detection> Inference::run_inference(cv::Mat& input)
{
  cv::Mat image_input = input.clone();

  cv::Mat blob;
  cv::dnn::blobFromImage(image_input, blob, 1.0/255.0, model_shape, cv::Scalar(), true, false);
  net.setInput(blob);

  std::vector<cv::Mat> outputs;
  net.forward(outputs, net.getUnconnectedOutLayersNames());

  // Detection
  cv::Mat boxes_output = outputs[0];

  int boxes_rows = boxes_output.size[2];    // 6300
  int boxes_dims = boxes_output.size[1];    // 116

  boxes_output = boxes_output.reshape(1, boxes_dims);
  cv::transpose(boxes_output, boxes_output);

  float x_factor = image_input.cols / model_shape.width;
  float y_factor = image_input.rows / model_shape.height;

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<int> indices;

  float* data = (float*) boxes_output.data;

  for (int i = 0; i < boxes_rows; i++)
  {
    cv::Mat scores(1, classes.size(), CV_32FC1, data + 4);

    cv::Point class_id;
    double max_class_score;
    minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

    if (max_class_score > *score_threshold)
    {
      indices.push_back(i);

      // limit values between 0 and image size
      float x1 = std::max(std::min(data[0], (float) model_shape.width), 0.0f);
      float y1 = std::max(std::min(data[1], (float) model_shape.height), 0.0f);
      float x2 = std::max(std::min(data[2], (float) model_shape.width), 0.0f);
      float y2 = std::max(std::min(data[3], (float) model_shape.height), 0.0f);

      int left = int(x1 * x_factor);
      int top = int(y1 * y_factor);
      int width = int((x2 - x1) * x_factor);
      int height = int((y2 - y1) * y_factor);

      boxes.push_back(cv::Rect(left, top, width, height));
      confidences.push_back(max_class_score);
      class_ids.push_back(class_id.x);
    }

    data += boxes_dims;
  }

  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, *score_threshold, *nms_threshold, nms_result);
  if (nms_result.empty()) return {};

  std::vector<Detection> detections(nms_result.size());
  for (size_t i = 0; i < nms_result.size(); i++)
  {
    int idx = nms_result[i];

    Detection result;
    result.class_id = class_ids[idx];
    result.confidence = confidences[idx];

    result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

    result.class_name = classes[result.class_id];
    result.box = boxes[idx];

    detections[i] = result;
  }

  // Segmentation
  if (outputs.size() > 1)
  {
    cv::Mat masks_output = outputs[1];
    int mask_proto = masks_output.size[1];   // 32
    int mask_height = masks_output.size[2];
    int mask_width = masks_output.size[3];

    float x_factor_segm = x_factor * model_shape.width / mask_width;
    float y_factor_segm = y_factor * model_shape.height / mask_height;

    data = (float*) boxes_output.data;
    cv::Mat masks_prediction(0, mask_proto, CV_32FC1);
    for (int idx : nms_result)
    {
      cv::Mat masks_temp(1, mask_proto, CV_32FC1, data + indices[idx] * boxes_dims + 4 + classes.size());
      cv::vconcat(masks_prediction, masks_temp, masks_prediction);
    }
    cv::transpose(masks_prediction, masks_prediction);

    masks_output = masks_output.reshape(1, {mask_proto, mask_height*mask_width});
    cv::transpose(masks_output, masks_output);

    cv::gemm(masks_output, masks_prediction, 1, cv::noArray(), 0, masks_output);

    // compute sigmoid of masks_output
    cv::exp(-masks_output, masks_output);
    cv::add(1.0, masks_output, masks_output);
    cv::divide(1.0, masks_output, masks_output);

    masks_output = masks_output.reshape((int) nms_result.size(), {mask_height, mask_width});
    std::vector<cv::Mat> images((int) nms_result.size());
    cv::split(masks_output, images);

    for (size_t i = 0; i < nms_result.size(); i++)
    {
      int x1 = (int) std::floor(boxes[i].x);
      int y1 = (int) std::floor(boxes[i].y);
      int x2 = (int) std::ceil(boxes[i].x + boxes[i].width);
      int y2 = (int) std::ceil(boxes[i].y + boxes[i].height);

      int scale_x1 = x1 / x_factor_segm;
      int scale_y1 = y1 / y_factor_segm;
      int scale_x2 = x2 / x_factor_segm;
      int scale_y2 = y2 / y_factor_segm;

      cv::Mat crop_mask = images[i].colRange(scale_x1, scale_x2).rowRange(scale_y1, scale_y2);

      cv::resize(crop_mask, crop_mask, cv::Size(x2 - x1, y2 - y1));

      cv::Size kernel_size = cv::Size(int(x_factor_segm) % 2 ? int(x_factor_segm) : int(x_factor_segm)+1,
                                      int(y_factor_segm) % 2 ? int(y_factor_segm) : int(y_factor_segm)+1);
      cv::GaussianBlur(crop_mask, crop_mask, kernel_size, 0);
      cv::threshold(crop_mask, crop_mask, 0.5, 1, cv::THRESH_BINARY);

      detections[i].mask = crop_mask;
    }
  }

  return detections;
}

void Inference::load_onnx_network()
{
  net = cv::dnn::readNetFromONNX(model_path);

  if (cuda_enabled)
  {
    std::cout << "\nRunning on CUDA" << std::endl;
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
  }
  else
  {
    std::cout << "\nRunning on CPU" << std::endl;
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
}

} // namespace ObjectDetector
