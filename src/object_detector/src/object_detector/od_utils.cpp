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

Inference::Inference(std::string& onnxModelPath,
                     cv::Size& modelInputShape,
                     bool runWithCuda,
                     double* scoreThreshold,
                     double* NMSthreshold)
{
  this->modelPath = onnxModelPath;
  this->modelShape = modelInputShape;
  this->cudaEnabled = runWithCuda;
  this->scoreThreshold = scoreThreshold;
  this->NMSthreshold = NMSthreshold;

  std::random_device rd;
  gen = std::mt19937(rd());
  dis = std::uniform_int_distribution<int>(100, 255);

  loadOnnxNetwork();
}

std::vector<Detection> Inference::runInference(cv::Mat &input)
{
  cv::Mat modelInput = input;
  if (letterBoxForSquare && modelShape.width == modelShape.height)
    modelInput = formatToSquare(modelInput);

  cv::Mat blob;
  cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
  net.setInput(blob);

  std::vector<cv::Mat> outputs;
  net.forward(outputs, net.getUnconnectedOutLayersNames());

  int rows = outputs[0].size[2];
  int dimensions = outputs[0].size[1];
  // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])

  outputs[0] = outputs[0].reshape(1, dimensions);
  cv::transpose(outputs[0], outputs[0]);

  float* data = (float*) outputs[0].data;

  float x_factor = modelInput.cols / modelShape.width;
  float y_factor = modelInput.rows / modelShape.height;

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for (int i = 0; i < rows; i++)
  {
    float *classes_scores = data+4;

    cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
    cv::Point class_id;
    double maxClassScore;

    minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

    if (maxClassScore > *scoreThreshold)
    {
      confidences.push_back(maxClassScore);
      class_ids.push_back(class_id.x);

      float x1 = data[0];
      float y1 = data[1];
      float x2 = data[2];
      float y2 = data[3];

      int left = int(x1 * x_factor);
      int top = int(y1 * y_factor);

      int width = int((x2 - x1) * x_factor);
      int height = int((y2 - y1) * y_factor);

      boxes.push_back(cv::Rect(left, top, width, height));
    }

    data += dimensions;
  }

  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, *scoreThreshold, *NMSthreshold, nms_result);

  std::vector<Detection> detections{};
  for (unsigned long i = 0; i < nms_result.size(); ++i)
  {
    int idx = nms_result[i];

    Detection result;
    result.class_id = class_ids[idx];
    result.confidence = confidences[idx];

    result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

    result.className = classes[result.class_id];
    result.box = boxes[idx];

    detections.push_back(result);
  }

  return detections;
}

void Inference::loadOnnxNetwork()
{
  net = cv::dnn::readNetFromONNX(modelPath);

  if (cudaEnabled)
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

cv::Mat Inference::formatToSquare(const cv::Mat &source)
{
  int col = source.cols;
  int row = source.rows;
  int _max = MAX(col, row);
  cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
  source.copyTo(result(cv::Rect(0, 0, col, row)));
  return result;
}

} // namespace ObjectDetector
