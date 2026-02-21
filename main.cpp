#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <mutex>
#include <filesystem>
#include <algorithm>

#include <cstdlib>
#include <fstream>
#include <cstring>
#include <unordered_set>

#include "config/configprocessor.h"
#include "network/networkcontroller.h"
#include "modules/battery/battery_state_node.h"
#include "modules/map/map_node.h"
#include "modules/basecontrol/basecontrol.h"
#include "modules/nav2/robot_navigator.h"

using namespace Robot;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

std::string loadVideoPipelineFromConfig(const std::string &config_path)
{
  std::ifstream file(config_path);
  if (!file.good()) {
    return "";
  }
  file.close();

  ConfigProcessor config(config_path);
  std::string src;
  std::string coding;
  std::string sink;

  if (config.searchString("Amur.gstreamer.src", src) != EXIT_SUCCESS) {
    return "";
  }
  if (config.searchString("Amur.gstreamer.coding", coding) != EXIT_SUCCESS) {
    return "";
  }
  if (config.searchString("Amur.gstreamer.sink", sink) != EXIT_SUCCESS) {
    return "";
  }

  return src + coding + sink;
}

std::string resolveVideoPipeline()
{
  const char *env_pipeline = std::getenv("RC_AGENT_GSTREAMER_PIPELINE");
  if (env_pipeline && *env_pipeline != '\0') {
    return std::string(env_pipeline);
  }

  return loadVideoPipelineFromConfig("robot.cfg");
}

}  // namespace

/**
 * @brief Node that publishes the last connected IP address.
 *
 * The AddressSender node publishes a string message containing the last connected IP address
 * retrieved from the NetworkController at a rate of one message per second.
 */
class AddressSender : public rclcpp::Node {
public:
  /**
   * @brief Constructs a new AddressSender node.
   *
   * @param network Shared pointer to a NetworkController instance used to obtain the IP address.
   */
  AddressSender(std::shared_ptr<NetworkController> network) :
  network_(network),
  Node("address_sender")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("remote_control_station/address", 10);

    // Start a timer to publish a message every second
    publish_timer_ = this->create_wall_timer(1s, std::bind(&AddressSender::publish_callback, this));
  }

private:
  /**
   * @brief Timer callback to publish the last connected IP address.
   *
   * This function retrieves the last connected IP from the NetworkController and publishes
   * it as a std_msgs::msg::String message.
   */
  void publish_callback() {
    message_.data = network_->getLastConnectedIP();
    publisher_->publish(message_);
    RCLCPP_DEBUG(this->get_logger(), "Publishing message: %s", message_.data.c_str());
  }

  std::shared_ptr<NetworkController> network_;                        ///< Shared pointer to the NetworkController.
  std_msgs::msg::String message_;                                     ///< Message object used for publishing the IP address.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;     ///< Publisher for the IP address message.
  rclcpp::TimerBase::SharedPtr publish_timer_;                        ///< Timer for periodic publishing.
};

class StreamPipelineReceiver : public rclcpp::Node {
public:
  StreamPipelineReceiver(std::shared_ptr<Sensors> sensors)
  : Node("stream_pipeline_receiver"),
    sensors_(std::move(sensors))
  {
    pipeline_topic_ = declare_parameter<std::string>("pipeline_topic", "/gst/pipeline");
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    sub_ = this->create_subscription<std_msgs::msg::String>(
      pipeline_topic_,
      qos,
      std::bind(&StreamPipelineReceiver::onPipeline, this, std::placeholders::_1));
  }

private:
  void onPipeline(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!sensors_ || msg->data.empty()) {
      return;
    }

    if (msg->data == last_pipeline_) {
      return;
    }

    last_pipeline_ = msg->data;
    sensors_->mutable_video_stream()->set_pipeline(last_pipeline_);
    RCLCPP_INFO(this->get_logger(), "Updated video pipeline from %s", pipeline_topic_.c_str());
  }

  std::shared_ptr<Sensors> sensors_;
  std::string pipeline_topic_;
  std::string last_pipeline_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

class VideoSourceRelay : public rclcpp::Node {
public:
  explicit VideoSourceRelay(const std::string &output_topic)
  : Node("video_source_relay"),
    output_topic_(output_topic.empty() ? "/rc_agent/video_source" : output_topic)
  {
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      output_topic_, rclcpp::SensorDataQoS());
  }

  void updateSource(const std::string &topic)
  {
    if (topic.empty() || topic == source_topic_) {
      return;
    }
    if (topic == output_topic_) {
      RCLCPP_WARN(get_logger(), "Ignoring relay source %s (matches output topic)",
                  topic.c_str());
      return;
    }
    source_topic_ = topic;
    warned_for_source_ = false;
    subscription_.reset();
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      source_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        publisher_->publish(normalizeImage(*msg));
      });
    RCLCPP_INFO(get_logger(), "Relaying video source %s -> %s",
                source_topic_.c_str(), output_topic_.c_str());
  }

  void updateTargetFormat(int width, int height, const std::string &encoding)
  {
    std::lock_guard<std::mutex> lock(format_mutex_);
    target_width_ = width;
    target_height_ = height;
    target_encoding_ = encoding;
  }

  const std::string &outputTopic() const
  {
    return output_topic_;
  }

private:
  static size_t bytesPerPixel(const std::string &encoding)
  {
    if (encoding == sensor_msgs::image_encodings::RGB8 ||
        encoding == sensor_msgs::image_encodings::BGR8) {
      return 3;
    }
    if (encoding == sensor_msgs::image_encodings::RGBA8 ||
        encoding == sensor_msgs::image_encodings::BGRA8) {
      return 4;
    }
    if (encoding == sensor_msgs::image_encodings::MONO8 ||
        encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
      return 1;
    }
    if (encoding == sensor_msgs::image_encodings::MONO16 ||
        encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      return 2;
    }
    if (encoding == sensor_msgs::image_encodings::YUV422 ||
        encoding == sensor_msgs::image_encodings::YUV422_YUY2) {
      return 2;
    }
    if (encoding == "uyvy" || encoding == "yuy2") {
      return 2;
    }
    return 0;
  }

  static bool isSupportedEncoding(const std::string &encoding)
  {
    return encoding == sensor_msgs::image_encodings::RGB8 ||
           encoding == sensor_msgs::image_encodings::BGR8 ||
           encoding == sensor_msgs::image_encodings::MONO8 ||
           encoding == sensor_msgs::image_encodings::RGBA8 ||
           encoding == sensor_msgs::image_encodings::BGRA8 ||
           encoding == sensor_msgs::image_encodings::MONO16 ||
           encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
           encoding == sensor_msgs::image_encodings::YUV422 ||
           encoding == sensor_msgs::image_encodings::YUV422_YUY2 ||
           encoding == "uyvy" || encoding == "yuy2";
  }

  sensor_msgs::msg::Image normalizeImage(const sensor_msgs::msg::Image &msg)
  {
    size_t bpp = bytesPerPixel(msg.encoding);
    if (bpp == 0 || msg.width == 0 || msg.height == 0) {
      return msg;
    }

    if (!isSupportedEncoding(msg.encoding)) {
      warnConversion(msg.encoding, "unsupported");
      return msg;
    }

    if (!isGscamNative(msg.encoding)) {
      warnConversion(msg.encoding, "converting to rgb8");
    }

    return convertAndScale(msg);
  }

  static bool isGscamNative(const std::string &encoding)
  {
    return encoding == sensor_msgs::image_encodings::RGB8 ||
           encoding == sensor_msgs::image_encodings::MONO8 ||
           encoding == sensor_msgs::image_encodings::YUV422_YUY2;
  }

  void warnConversion(const std::string &encoding, const char *reason)
  {
    std::lock_guard<std::mutex> lock(warn_mutex_);
    if (warned_for_source_) {
      return;
    }
    warned_for_source_ = true;
    RCLCPP_WARN(get_logger(),
                "Video source '%s' encoding '%s' (%s); converting to rgb8",
                source_topic_.c_str(),
                encoding.c_str(),
                reason);
  }

  static sensor_msgs::msg::Image convertAndScale(const sensor_msgs::msg::Image &msg)
  {
    const bool is_bgr = msg.encoding == sensor_msgs::image_encodings::BGR8;
    const bool is_rgba = msg.encoding == sensor_msgs::image_encodings::RGBA8;
    const bool is_bgra = msg.encoding == sensor_msgs::image_encodings::BGRA8;
    const bool is_mono = msg.encoding == sensor_msgs::image_encodings::MONO8;
    const bool is_mono16 = msg.encoding == sensor_msgs::image_encodings::MONO16 ||
                           msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
    const bool is_yuy2 = msg.encoding == sensor_msgs::image_encodings::YUV422_YUY2 ||
                         msg.encoding == "yuy2";
    const bool is_uyvy = msg.encoding == sensor_msgs::image_encodings::YUV422 ||
                         msg.encoding == "uyvy";
    const size_t src_bpp = is_mono ? 1 : 3;
    const size_t src_stride = (is_rgba || is_bgra) ? 4 : (is_mono || is_mono16 ? 1 : 3);
    const size_t src_expected_step = msg.width * src_bpp;
    if (msg.step < src_expected_step) {
      return msg;
    }

    int dst_width = static_cast<int>(msg.width);
    int dst_height = static_cast<int>(msg.height);
    {
      std::lock_guard<std::mutex> lock(format_mutex_);
      if (target_width_ > 0) {
        dst_width = target_width_;
      }
      if (target_height_ > 0) {
        dst_height = target_height_;
      }
    }
    if (dst_width <= 0 || dst_height <= 0) {
      return msg;
    }

    sensor_msgs::msg::Image out = msg;
    out.encoding = sensor_msgs::image_encodings::RGB8;
    out.width = static_cast<uint32_t>(dst_width);
    out.height = static_cast<uint32_t>(dst_height);
    out.step = static_cast<sensor_msgs::msg::Image::_step_type>(dst_width * 3);
    out.data.resize(out.step * dst_height);

    const int src_width = static_cast<int>(msg.width);
    const int src_height = static_cast<int>(msg.height);
    const double scale_x = (dst_width > 1 && src_width > 1)
                             ? static_cast<double>(src_width - 1) / (dst_width - 1)
                             : 0.0;
    const double scale_y = (dst_height > 1 && src_height > 1)
                             ? static_cast<double>(src_height - 1) / (dst_height - 1)
                             : 0.0;

    auto sample_rgb = [&](int sx, int sy, uint8_t *rgb) {
      const uint8_t *row = msg.data.data() + static_cast<size_t>(sy) * msg.step;
      const uint8_t *px = row + static_cast<size_t>(sx) * src_stride;
      if (is_mono) {
        rgb[0] = px[0];
        rgb[1] = px[0];
        rgb[2] = px[0];
      } else if (is_mono16) {
        const uint8_t hi = px[1];
        rgb[0] = hi;
        rgb[1] = hi;
        rgb[2] = hi;
      } else if (is_rgba) {
        rgb[0] = px[0];
        rgb[1] = px[1];
        rgb[2] = px[2];
      } else if (is_bgra) {
        rgb[0] = px[2];
        rgb[1] = px[1];
        rgb[2] = px[0];
      } else if (is_bgr) {
        rgb[0] = px[2];
        rgb[1] = px[1];
        rgb[2] = px[0];
      } else if (is_yuy2 || is_uyvy) {
        const int pair = (sx / 2) * 4;
        const uint8_t *p = row + pair;
        uint8_t y = 0;
        uint8_t u = 0;
        uint8_t v = 0;
        if (is_yuy2) {
          y = (sx % 2 == 0) ? p[0] : p[2];
          u = p[1];
          v = p[3];
        } else {
          y = (sx % 2 == 0) ? p[1] : p[3];
          u = p[0];
          v = p[2];
        }
        const double yf = static_cast<double>(y);
        const double uf = static_cast<double>(u) - 128.0;
        const double vf = static_cast<double>(v) - 128.0;
        double r = yf + 1.402 * vf;
        double g = yf - 0.344136 * uf - 0.714136 * vf;
        double b = yf + 1.772 * uf;
        rgb[0] = static_cast<uint8_t>(std::clamp(r, 0.0, 255.0));
        rgb[1] = static_cast<uint8_t>(std::clamp(g, 0.0, 255.0));
        rgb[2] = static_cast<uint8_t>(std::clamp(b, 0.0, 255.0));
      } else {
        rgb[0] = px[0];
        rgb[1] = px[1];
        rgb[2] = px[2];
      }
    };

    for (int y = 0; y < dst_height; ++y) {
      double fy = scale_y * y;
      int y0 = static_cast<int>(fy);
      int y1 = std::min(y0 + 1, src_height - 1);
      double wy = fy - y0;
      uint8_t *dst_row = out.data.data() + static_cast<size_t>(y) * out.step;
      for (int x = 0; x < dst_width; ++x) {
        double fx = scale_x * x;
        int x0 = static_cast<int>(fx);
        int x1 = std::min(x0 + 1, src_width - 1);
        double wx = fx - x0;
        uint8_t c00[3], c01[3], c10[3], c11[3];
        sample_rgb(x0, y0, c00);
        sample_rgb(x1, y0, c01);
        sample_rgb(x0, y1, c10);
        sample_rgb(x1, y1, c11);
        uint8_t *dst_px = dst_row + static_cast<size_t>(x) * 3;
        for (int c = 0; c < 3; ++c) {
          double top = (1.0 - wx) * c00[c] + wx * c01[c];
          double bot = (1.0 - wx) * c10[c] + wx * c11[c];
          double val = (1.0 - wy) * top + wy * bot;
          dst_px[c] = static_cast<uint8_t>(std::clamp(val, 0.0, 255.0));
        }
      }
    }

    return out;
  }

  std::string output_topic_;
  std::string source_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::mutex format_mutex_;
  int target_width_{0};
  int target_height_{0};
  std::string target_encoding_;
  std::mutex warn_mutex_;
  bool warned_for_source_{false};
};

class VideoControlNode : public rclcpp::Node {
public:
  VideoControlNode(std::shared_ptr<Controls> controls,
                   std::shared_ptr<Sensors> sensors,
                   std::shared_ptr<NetworkController> network,
                   std::shared_ptr<VideoSourceRelay> relay)
  : Node("video_control"),
    controls_(std::move(controls)),
    sensors_(std::move(sensors)),
    network_(std::move(network)),
    relay_(std::move(relay))
  {
    stream_host_ = declare_parameter<std::string>("stream_host", "127.0.0.1");
    stream_port_ = declare_parameter<int>("stream_port", 5600);
    default_width_ = declare_parameter<int>("default_width", 640);
    default_height_ = declare_parameter<int>("default_height", 480);
    default_fps_ = declare_parameter<double>("default_fps", 30.0);
    default_bitrate_kbps_ = declare_parameter<int>("default_bitrate_kbps", 800);
    gscam_node_name_ = declare_parameter<std::string>("gscam_node_name", "gscam_publisher");
    relay_topic_ = declare_parameter<std::string>("relay_topic", "/rc_agent/video_source");

    update_timer_ = create_wall_timer(1s, std::bind(&VideoControlNode::onTimer, this));
  }

  void initParamsClient()
  {
    if (!gscam_params_) {
      gscam_params_ = std::make_shared<rclcpp::AsyncParametersClient>(shared_from_this(), gscam_node_name_);
    }
  }

private:
  static bool sameSource(const video::VideoSource &a, const video::VideoSource &b)
  {
    return a.type() == b.type() && a.name() == b.name();
  }

  static bool sameFormat(const video::VideoFormat &a, const video::VideoFormat &b)
  {
    return a.width() == b.width() &&
           a.height() == b.height() &&
           a.fps() == b.fps() &&
           a.encoding() == b.encoding() &&
           a.bitrate_kbps() == b.bitrate_kbps();
  }

  std::vector<video::VideoSource> collectSources()
  {
    std::vector<video::VideoSource> sources;
    auto topics = this->get_topic_names_and_types();
    for (const auto &entry : topics) {
      const auto &name = entry.first;
      if (!relay_topic_.empty() && name == relay_topic_) {
        continue;
      }
      const auto &types = entry.second;
      for (const auto &type : types) {
        if (type.find("sensor_msgs/msg/Image") != std::string::npos) {
          video::VideoSource src;
          src.set_type(video::ROS_IMAGE_TOPIC);
          src.set_name(name);
          sources.push_back(src);
          break;
        }
      }
    }

    try {
      for (const auto &entry : fs::directory_iterator("/dev")) {
        const auto filename = entry.path().filename().string();
        if (filename.rfind("video", 0) == 0) {
          video::VideoSource src;
          src.set_type(video::V4L2_DEVICE);
          src.set_name(entry.path().string());
          sources.push_back(src);
        }
      }
    } catch (const fs::filesystem_error &e) {
      RCLCPP_WARN(get_logger(), "Failed to enumerate /dev: %s", e.what());
    }

    return sources;
  }

  video::VideoSource pickDefaultSource(const std::vector<video::VideoSource> &sources) const
  {
    if (!sources.empty()) {
      return sources.front();
    }
    video::VideoSource empty;
    empty.set_type(video::SOURCE_UNKNOWN);
    empty.set_name("");
    return empty;
  }

  video::VideoFormat buildFormat(const video::VideoFormat &requested) const
  {
    video::VideoFormat out = requested;
    if (out.width() == 0) {
      out.set_width(default_width_);
    }
    if (out.height() == 0) {
      out.set_height(default_height_);
    }
    if (out.fps() <= 0.0f) {
      out.set_fps(static_cast<float>(default_fps_));
    }
    if (out.bitrate_kbps() == 0) {
      out.set_bitrate_kbps(default_bitrate_kbps_);
    }
    return out;
  }

  std::string buildPipeline(const video::VideoSource &source, const video::VideoFormat &format) const
  {
    int fps = static_cast<int>(format.fps() > 0.0f ? format.fps() : default_fps_);
    int width = format.width() > 0 ? static_cast<int>(format.width()) : default_width_;
    int height = format.height() > 0 ? static_cast<int>(format.height()) : default_height_;
    int bitrate = format.bitrate_kbps() > 0 ? static_cast<int>(format.bitrate_kbps()) : default_bitrate_kbps_;
    std::string host = stream_host_;
    if (host.empty() || host == "127.0.0.1") {
      if (network_) {
        const auto last_ip = network_->getLastConnectedIP();
        if (!last_ip.empty()) {
          host = last_ip;
        }
      }
    }
    if (host.empty()) {
      host = "127.0.0.1";
    }

    std::string src;
    if (source.type() == video::V4L2_DEVICE) {
      src = "v4l2src device=" + source.name();
    } else {
      src = "rosimagesrc ros-topic=" + source.name();
    }

    return src +
           " ! videoconvert ! tee name=t" +
           " t. ! queue ! videoconvert ! videoscale method=1" +
           " ! video/x-raw,format=I420,width=" + std::to_string(width) +
           ",height=" + std::to_string(height) +
           ",framerate=" + std::to_string(fps) + "/1" +
           " ! x264enc tune=zerolatency bitrate=" + std::to_string(bitrate) +
           " speed-preset=ultrafast key-int-max=30" +
           " ! rtph264pay pt=96 config-interval=1" +
           " ! udpsink host=" + host + " port=" + std::to_string(stream_port_) +
           " sync=false async=false" +
           " t. ! queue ! videoconvert";
  }

  void updateStatus(const std::vector<video::VideoSource> &sources,
                    const video::VideoSource &active,
                    const video::VideoFormat &format,
                    std::shared_ptr<grpcClient> client)
  {
    if (!sensors_) {
      return;
    }

    std::unique_lock<std::mutex> lock;
    if (client) {
      lock = std::unique_lock<std::mutex>(client->getMutex());
    }
    auto status = sensors_->mutable_video_stream()->mutable_status();
    status->clear_sources();
    for (const auto &src : sources) {
      auto out = status->add_sources();
      out->set_type(src.type());
      out->set_name(src.name());
    }
    status->mutable_active_source()->set_type(active.type());
    status->mutable_active_source()->set_name(active.name());
    status->mutable_format()->set_width(format.width());
    status->mutable_format()->set_height(format.height());
    status->mutable_format()->set_fps(format.fps());
    status->mutable_format()->set_encoding(format.encoding());
    status->mutable_format()->set_bitrate_kbps(format.bitrate_kbps());
  }

  void applyConfig(const std::vector<video::VideoSource> &sources,
                   std::shared_ptr<grpcClient> client)
  {
    if (!controls_ || !sensors_) {
      return;
    }

    video::VideoConfig requested;
    bool has_request = false;
    {
      if (client) {
        std::unique_lock<std::mutex> lock(client->getMutex());
        if (controls_->has_video_config()) {
          requested = controls_->video_config();
          has_request = true;
        }
      } else if (controls_->has_video_config()) {
        requested = controls_->video_config();
        has_request = true;
      }
    }

    if (!has_request) {
      video::VideoSource active = last_source_;
      if (active.name().empty()) {
        active = pickDefaultSource(sources);
      }
      auto format = buildFormat(video::VideoFormat());
      if (relay_ && active.type() == video::ROS_IMAGE_TOPIC && !active.name().empty()) {
        relay_->updateSource(active.name());
        relay_->updateTargetFormat(
          static_cast<int>(format.width()),
          static_cast<int>(format.height()),
          format.encoding());
      }
      updateStatus(sources, active, format, client);
      return;
    }

    video::VideoSource desired = requested.requested_source();
    if (desired.name().empty()) {
      desired = pickDefaultSource(sources);
    } else if (!relay_topic_.empty() && desired.name() == relay_topic_) {
      desired = pickDefaultSource(sources);
    }

    video::VideoFormat format = buildFormat(requested.format());
    video::VideoFormat effective_format = format;
    if (desired.type() == video::ROS_IMAGE_TOPIC && relay_) {
      effective_format.set_width(default_width_);
      effective_format.set_height(default_height_);
      effective_format.set_encoding("rgb8");
    }
    if (desired.name().empty()) {
      if (relay_ && desired.type() == video::ROS_IMAGE_TOPIC && !desired.name().empty()) {
        relay_->updateSource(desired.name());
        relay_->updateTargetFormat(
          static_cast<int>(effective_format.width()),
          static_cast<int>(effective_format.height()),
          effective_format.encoding());
      }
      updateStatus(sources, desired, effective_format, client);
      return;
    }

    if (sameSource(desired, last_source_) && sameFormat(effective_format, last_format_)) {
      updateStatus(sources, last_source_, last_format_, client);
      return;
    }

    if (desired.type() == video::ROS_IMAGE_TOPIC && relay_) {
      relay_->updateSource(desired.name());
      relay_->updateTargetFormat(
        static_cast<int>(effective_format.width()),
        static_cast<int>(effective_format.height()),
        effective_format.encoding());
      if (using_relay_pipeline_) {
        last_source_ = desired;
        last_format_ = effective_format;
        updateStatus(sources, last_source_, last_format_, client);
        return;
      }
    }

    if (!gscam_params_ || !gscam_params_->wait_for_service(500ms)) {
      updateStatus(sources, desired, format, client);
      return;
    }

    std::string pipeline = requested.pipeline_override();
    if (pipeline.empty()) {
      if (desired.type() == video::ROS_IMAGE_TOPIC && relay_) {
        pipeline = buildPipeline(relaySource(), format);
        using_relay_pipeline_ = true;
      } else {
        pipeline = buildPipeline(desired, format);
        using_relay_pipeline_ = false;
      }
    }

    gscam_params_->set_parameters(
      {rclcpp::Parameter("gscam_config", pipeline)});

    last_source_ = desired;
    last_format_ = effective_format;
    updateStatus(sources, last_source_, last_format_, client);
  }

  void onTimer()
  {
    initParamsClient();
    auto sources = collectSources();
    std::shared_ptr<grpcClient> client;
    if (network_) {
      client = network_->getClientInstance();
    }
    applyConfig(sources, client);
  }

  std::shared_ptr<Controls> controls_;
  std::shared_ptr<Sensors> sensors_;
  std::shared_ptr<NetworkController> network_;
  std::shared_ptr<VideoSourceRelay> relay_;
  std::shared_ptr<rclcpp::AsyncParametersClient> gscam_params_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::string stream_host_;
  int stream_port_{5600};
  int default_width_{640};
  int default_height_{480};
  double default_fps_{30.0};
  int default_bitrate_kbps_{800};
  std::string gscam_node_name_;
  std::string relay_topic_;
  video::VideoSource last_source_;
  video::VideoFormat last_format_;
  bool using_relay_pipeline_{true};

  video::VideoSource relaySource() const
  {
    video::VideoSource relay_src;
    relay_src.set_type(video::ROS_IMAGE_TOPIC);
    relay_src.set_name(relay_topic_);
    return relay_src;
  }
};

/**
 * @brief Main function that initializes ROS 2 nodes and runs the executor.
 *
 * The main function initializes the ROS 2 environment, creates several nodes (including the network controller,
 * map node, and navigator node), waits for a connection to the server, and then spins a multi-threaded executor.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit status.
 */
int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  if (!rclcpp::ok()) {
    std::cerr << "ROS2 initialization failed!" << std::endl;
    return 1;
  }

  auto paramNode = std::make_shared<rclcpp::Node>("rc_agent");
  const std::string param_pipeline =
    paramNode->declare_parameter<std::string>("video_pipeline", "");

  // Create shared pointers for Controls, Sensors, and Map data
  auto controlsPtr = std::make_shared<Controls>();
  auto sensorsPtr = std::make_shared<Sensors>();
  auto mapPtr = std::make_shared<GetMapResponse>();
  std::string video_pipeline = param_pipeline;
  if (video_pipeline.empty()) {
    video_pipeline = resolveVideoPipeline();
  }
  if (!video_pipeline.empty()) {
    sensorsPtr->mutable_video_stream()->set_pipeline(video_pipeline);
  }

  // Network ports from parameters
  int bcastPort = paramNode->declare_parameter<int>("network.bcast_port", 11111);
  int arpingPort = paramNode->declare_parameter<int>("network.arping_port", 11112);

  // Create a NetworkController node and start the arping service
  std::shared_ptr<NetworkController> network = std::make_shared<NetworkController>(controlsPtr, sensorsPtr, mapPtr);
  network->startArpingService(bcastPort, arpingPort);

  // Create an AddressSender node that publishes the last connected IP address
  auto addressSenderNode = std::make_shared<AddressSender>(network);
  auto pipelineReceiverNode = std::make_shared<StreamPipelineReceiver>(sensorsPtr);
  auto relayNode = std::make_shared<VideoSourceRelay>("/rc_agent/video_source");
  auto videoControlNode = std::make_shared<VideoControlNode>(
    controlsPtr, sensorsPtr, network, relayNode);
  videoControlNode->initParamsClient();

  // Wait until the client successfully connects to the server
  while(true){
    if (!rclcpp::ok())
      return 0;

    if(network->runClient() == 0){
      std::cout << "Connected to server" << std::endl;
      network->stopArpingService();
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Wait 10 milliseconds
  }

  // Initialize additional nodes (commented out nodes can be enabled if needed).
//   auto batteryStateNode = std::make_shared<BatteryStateNode>(sensorsPtr->mutable_batterystate(), "battery_state");
  auto mapNode = std::make_shared<MapNode>(mapPtr, "/map", "/mapannotator/rc_agent_zones",
                                           network->getClientInstance()->getMapMutex());
  auto baseControlNode = std::make_shared<BaseControlNode>(
    controlsPtr->mutable_basecontrol(),
    network->getClientInstance()->getMutex());
  auto navigatorNode = std::make_shared<RobotNavigator>(
    "basic_navigator_node",
    std::shared_ptr<NavCommandRequest>(controlsPtr->mutable_navcontrol(), [](NavCommandRequest*){}),
    std::shared_ptr<NavCommandResponse>(sensorsPtr->mutable_navcontrolstatus(), [](NavCommandResponse*){}),
    network->getClientInstance()->getMutex()
  );

  // Create a MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add the nodes to the executor
  executor.add_node(addressSenderNode);
  executor.add_node(pipelineReceiverNode);
  executor.add_node(relayNode);
  executor.add_node(videoControlNode);
//   executor.add_node(batteryStateNode);
  executor.add_node(mapNode);
  executor.add_node(baseControlNode);
  executor.add_node(navigatorNode);

  // Spin the executor to keep the nodes running
  executor.spin();

  // Shutdown the ROS 2 environment
  rclcpp::shutdown();

  return 0;
}
