#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

struct QrDetection
{
  std::string text;
  std::vector<cv::Point2f> points;
  int center_x = 0;
  int center_y = 0;
  int offset_x = 0;
  int offset_y = 0;
  int area = 0;
};

struct QrTask
{
  bool valid = false;
  std::string class1;
  std::string class2;
  std::string landing_side;
};

class QrDetectorNode : public rclcpp::Node
{
public:
  QrDetectorNode() : Node("qr_detector_cpp")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/image");
    text_topic_ = declare_parameter<std::string>("text_topic", "/qr_code/text");
    offset_topic_ = declare_parameter<std::string>("offset_topic", "/qr_code/offset");
    result_topic_ = declare_parameter<std::string>("json_topic", "/qr_code/result");
    task_topic_ = declare_parameter<std::string>("task_topic", "/qr_task");
    min_area_ = declare_parameter<int>("min_area", 100);
    allow_empty_ = declare_parameter<bool>("allow_empty", false);
    confirm_frames_ = declare_parameter<int>("confirm_frames", 3);
    log_text_ = declare_parameter<bool>("log_text", false);
    log_every_frame_ = declare_parameter<bool>("log_every_frame", false);

    text_pub_ = create_publisher<std_msgs::msg::String>(text_topic_, 10);
    offset_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(offset_topic_, 10);
    result_pub_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);
    task_pub_ = create_publisher<std_msgs::msg::String>(task_topic_, 10);

    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_, 10,
      std::bind(&QrDetectorNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "qr_detector_cpp subscribed to %s; publishing %s, %s, %s, %s",
      image_topic_.c_str(), text_topic_.c_str(), offset_topic_.c_str(),
      result_topic_.c_str(), task_topic_.c_str());
  }

private:
  static std::string trim(const std::string & value)
  {
    size_t begin = 0;
    while (begin < value.size() &&
      std::isspace(static_cast<unsigned char>(value[begin])))
    {
      ++begin;
    }

    size_t end = value.size();
    while (end > begin &&
      std::isspace(static_cast<unsigned char>(value[end - 1])))
    {
      --end;
    }

    return value.substr(begin, end - begin);
  }

  static std::string toLower(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    return value;
  }

  static std::string jsonEscape(const std::string & value)
  {
    std::ostringstream out;
    for (const unsigned char ch : value) {
      switch (ch) {
        case '\\':
          out << "\\\\";
          break;
        case '"':
          out << "\\\"";
          break;
        case '\n':
          out << "\\n";
          break;
        case '\r':
          out << "\\r";
          break;
        case '\t':
          out << "\\t";
          break;
        default:
          if (ch < 0x20) {
            out << "\\u00";
            const char * hex = "0123456789abcdef";
            out << hex[(ch >> 4) & 0x0f] << hex[ch & 0x0f];
          } else {
            out << ch;
          }
          break;
      }
    }
    return out.str();
  }

  static void replaceAll(std::string & value, const std::string & from, const std::string & to)
  {
    size_t pos = 0;
    while ((pos = value.find(from, pos)) != std::string::npos) {
      value.replace(pos, from.size(), to);
      pos += to.size();
    }
  }

  static bool isIgnoredTaskKey(const std::string & token)
  {
    return token == "class" || token == "class1" || token == "class2" ||
      token == "target" || token == "target1" || token == "target2" ||
      token == "side" || token == "landing" || token == "landing_side" ||
      token == "land" || token == "qr" || token == "task";
  }

  static std::vector<std::string> splitTaskTokens(std::string text)
  {
    replaceAll(text, "\xef\xbc\x8c", ",");
    replaceAll(text, "\xe3\x80\x81", ",");
    replaceAll(text, "\xef\xbc\x9a", ":");
    replaceAll(text, "\xef\xbc\x9b", ";");

    for (char & ch : text) {
      if (ch == ',' || ch == ';' || ch == '|' || ch == ':' || ch == '=' ||
        ch == '/' || ch == '\\' || ch == '[' || ch == ']' || ch == '{' ||
        ch == '}' || ch == '(' || ch == ')' || ch == '"' || ch == '\'')
      {
        ch = ' ';
      }
    }

    std::istringstream in(text);
    std::vector<std::string> tokens;
    std::string token;
    while (in >> token) {
      token = trim(token);
      if (!token.empty()) {
        tokens.push_back(token);
      }
    }
    return tokens;
  }

  static QrTask parseTaskText(const std::string & text)
  {
    QrTask task;
    const auto tokens = splitTaskTokens(text);
    std::vector<std::string> classes;

    for (const auto & raw_token : tokens) {
      const std::string token = trim(raw_token);
      const std::string lower = toLower(token);
      if (lower == "left" || lower == "right") {
        task.landing_side = lower;
        continue;
      }

      if (!isIgnoredTaskKey(lower)) {
        classes.push_back(token);
      }
    }

    if (classes.size() >= 2 && !task.landing_side.empty()) {
      task.valid = true;
      task.class1 = classes[0];
      task.class2 = classes[1];
    }
    return task;
  }

  static std::string taskToJson(const QrTask & task, bool confirmed, int stable_frames)
  {
    std::ostringstream out;
    out << "{\"valid\":" << (task.valid ? "true" : "false")
        << ",\"confirmed\":" << (confirmed ? "true" : "false")
        << ",\"stable_frames\":" << stable_frames
        << ",\"class1\":\"" << jsonEscape(task.class1)
        << "\",\"class2\":\"" << jsonEscape(task.class2)
        << "\",\"landing_side\":\"" << jsonEscape(task.landing_side) << "\"}";
    return out.str();
  }

  static std::vector<cv::Point2f> pointsFromMat(const cv::Mat & points)
  {
    std::vector<cv::Point2f> out;
    if (points.empty()) {
      return out;
    }

    const cv::Mat flat = points.reshape(2, static_cast<int>(points.total()));
    out.reserve(static_cast<size_t>(flat.rows));
    for (int i = 0; i < flat.rows; ++i) {
      out.push_back(flat.at<cv::Point2f>(i));
    }
    return out;
  }

  static int contourArea(const std::vector<cv::Point2f> & points)
  {
    if (points.size() < 4) {
      return 0;
    }
    return static_cast<int>(std::abs(cv::contourArea(points)));
  }

  std::vector<QrDetection> detectQrCodes(const cv::Mat & frame)
  {
    std::vector<QrDetection> detections;
    if (frame.empty()) {
      return detections;
    }

    std::vector<std::string> decoded_info;
    cv::Mat multi_points;
    const bool multi_ok = detector_.detectAndDecodeMulti(frame, decoded_info, multi_points);
    if (multi_ok && !multi_points.empty()) {
      for (int i = 0; i < multi_points.rows; ++i) {
        cv::Mat row = multi_points.row(i).clone();
        const std::string text = i < static_cast<int>(decoded_info.size()) ?
          decoded_info[static_cast<size_t>(i)] : "";
        buildDetection(frame, text, row, detections);
      }
    }

    if (!detections.empty()) {
      return detections;
    }

    cv::Mat points;
    const std::string text = detector_.detectAndDecode(frame, points);
    buildDetection(frame, text, points, detections);
    return detections;
  }

  void buildDetection(
    const cv::Mat & frame,
    const std::string & text,
    const cv::Mat & points,
    std::vector<QrDetection> & detections) const
  {
    if (!allow_empty_ && text.empty()) {
      return;
    }

    std::vector<cv::Point2f> pts = pointsFromMat(points);
    if (pts.size() < 4) {
      return;
    }

    const int area = contourArea(pts);
    if (area < min_area_) {
      return;
    }

    cv::Point2f center(0.0f, 0.0f);
    for (const auto & point : pts) {
      center += point;
    }
    center *= 1.0f / static_cast<float>(pts.size());

    QrDetection detection;
    detection.text = text;
    detection.points = std::move(pts);
    detection.center_x = static_cast<int>(std::lround(center.x));
    detection.center_y = static_cast<int>(std::lround(center.y));
    detection.offset_x = detection.center_x - frame.cols / 2;
    detection.offset_y = detection.center_y - frame.rows / 2;
    detection.area = area;
    detections.push_back(std::move(detection));
  }

  cv::Mat decodeImage(const sensor_msgs::msg::CompressedImage & msg) const
  {
    if (msg.data.empty()) {
      return {};
    }

    const std::vector<unsigned char> buffer(msg.data.begin(), msg.data.end());
    return cv::imdecode(buffer, cv::IMREAD_COLOR);
  }

  void publishNotFound()
  {
    std_msgs::msg::Int32MultiArray offset_msg;
    offset_msg.data = {0, 0, 0, 0, 0, 0};
    offset_pub_->publish(offset_msg);

    std_msgs::msg::String task_msg;
    task_msg.data = taskToJson(QrTask{}, false, 0);
    task_pub_->publish(task_msg);
  }

  QrDetection pickBestDetection(const std::vector<QrDetection> & detections) const
  {
    return *std::max_element(
      detections.begin(), detections.end(),
      [](const QrDetection & lhs, const QrDetection & rhs) {
        return lhs.area < rhs.area;
      });
  }

  std::string resultToJson(const QrDetection & detection, int count) const
  {
    std::ostringstream out;
    out << "{\"found\":true,\"count\":" << count
        << ",\"text\":\"" << jsonEscape(detection.text)
        << "\",\"center\":[" << detection.center_x << "," << detection.center_y
        << "],\"offset\":[" << detection.offset_x << "," << detection.offset_y
        << "],\"area\":" << detection.area << ",\"points\":[";

    for (size_t i = 0; i < detection.points.size(); ++i) {
      if (i > 0) {
        out << ",";
      }
      out << "[" << static_cast<int>(std::lround(detection.points[i].x))
          << "," << static_cast<int>(std::lround(detection.points[i].y)) << "]";
    }
    out << "]}";
    return out.str();
  }

  void updateStability(const std::string & text)
  {
    if (text.empty()) {
      stable_text_.clear();
      stable_frames_ = 0;
      return;
    }

    if (text == stable_text_) {
      ++stable_frames_;
    } else {
      stable_text_ = text;
      stable_frames_ = 1;
    }
  }

  void publishDetection(const QrDetection & detection, int count)
  {
    updateStability(detection.text);
    const QrTask task = parseTaskText(detection.text);
    const bool confirmed = task.valid && stable_frames_ >= std::max(confirm_frames_, 1);

    std_msgs::msg::String text_msg;
    text_msg.data = detection.text;
    text_pub_->publish(text_msg);

    std_msgs::msg::Int32MultiArray offset_msg;
    offset_msg.data = {
      1,
      detection.center_x,
      detection.center_y,
      detection.offset_x,
      detection.offset_y,
      detection.area,
    };
    offset_pub_->publish(offset_msg);

    std_msgs::msg::String result_msg;
    result_msg.data = resultToJson(detection, count);
    result_pub_->publish(result_msg);

    std_msgs::msg::String task_msg;
    task_msg.data = taskToJson(task, confirmed, stable_frames_);
    task_pub_->publish(task_msg);

    if (log_every_frame_ || detection.text != last_logged_text_) {
      if (log_text_) {
        RCLCPP_INFO(
          get_logger(),
          "QR text='%s' center=(%d,%d) offset=(%d,%d) area=%d task=%s",
          detection.text.c_str(), detection.center_x, detection.center_y,
          detection.offset_x, detection.offset_y, detection.area,
          task_msg.data.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "QR text=<decoded length=%zu> center=(%d,%d) offset=(%d,%d) area=%d task=%s",
          detection.text.size(), detection.center_x, detection.center_y,
          detection.offset_x, detection.offset_y, detection.area,
          task_msg.data.c_str());
      }
      last_logged_text_ = detection.text;
    }
  }

  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv::Mat frame = decodeImage(*msg);
    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to decode compressed image from %s", image_topic_.c_str());
      return;
    }

    const std::vector<QrDetection> detections = detectQrCodes(frame);
    if (detections.empty()) {
      updateStability("");
      publishNotFound();
      if (log_every_frame_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "No QR code detected");
      }
      return;
    }

    const QrDetection best = pickBestDetection(detections);
    publishDetection(best, static_cast<int>(detections.size()));
  }

  cv::QRCodeDetector detector_;
  std::string image_topic_;
  std::string text_topic_;
  std::string offset_topic_;
  std::string result_topic_;
  std::string task_topic_;
  int min_area_ = 100;
  bool allow_empty_ = false;
  int confirm_frames_ = 3;
  bool log_text_ = false;
  bool log_every_frame_ = false;
  std::string stable_text_;
  int stable_frames_ = 0;
  std::string last_logged_text_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr offset_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QrDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

$ ssh sunrise@172.20.10.2
ssh: connect to host 172.20.10.2 port 22: Host is down