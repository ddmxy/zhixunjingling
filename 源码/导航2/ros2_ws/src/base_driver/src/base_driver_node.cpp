#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr uint8_t kSof = 0xFF;
constexpr uint8_t kEof = 0xFE;
constexpr uint8_t kCmdDownstream = 0x02;
constexpr uint8_t kCmdVelFeedback = 0x03;
constexpr uint8_t kCmdYawFeedback = 0x04;
constexpr std::size_t kPayloadSize = 8;
constexpr std::size_t kFrameSize = 11;

double normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

class BaseDriverNode : public rclcpp::Node
{
public:
  BaseDriverNode()
  : Node("base_driver_node"),
    serial_fd_(-1),
    running_(true),
    state_(ParserState::kWaitSof),
    payload_index_(0)
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);
    use_first_yaw_as_zero_ = declare_parameter<bool>("use_first_yaw_as_zero", true);
    assume_nonholonomic_ = declare_parameter<bool>("assume_nonholonomic", true);
    vx_deadband_mps_ = declare_parameter<double>("vx_deadband_mps", 0.005);
    vy_deadband_mps_ = declare_parameter<double>("vy_deadband_mps", 0.01);
    wz_deadband_rps_ = declare_parameter<double>("wz_deadband_rps", 0.01);
    feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s", 0.20);
    // Software odom calibration (RDK side). Physical 360deg -> MCU yaw ~478deg, wz ~716deg.
    imu_yaw_delta_scale_ = declare_parameter<double>("imu_yaw_delta_scale", 360.0 / 478.0);
    imu_wz_scale_ = declare_parameter<double>("imu_wz_scale", 360.0 / 716.0);
    vx_scale_ = declare_parameter<double>("vx_scale", 1.0);
    cmd_watchdog_timeout_s_ = declare_parameter<double>("cmd_watchdog_timeout_s", 0.5);
    send_zero_cmd_on_startup_ = declare_parameter<bool>("send_zero_cmd_on_startup", true);
    zero_cmd_hz_ = declare_parameter<double>("zero_cmd_hz", 2.0);
    enable_rx_log_ = declare_parameter<bool>("enable_rx_log", true);

    pose_cov_diag_ = declare_parameter<std::vector<double>>(
      "pose_covariance_diag", {0.05, 0.05, 1e6, 1e6, 1e6, 0.2});
    twist_cov_diag_ = declare_parameter<std::vector<double>>(
      "twist_covariance_diag", {0.05, 0.05, 1e6, 1e6, 1e6, 0.2});

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 20);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 20,
      std::bind(&BaseDriverNode::onCmdVel, this, std::placeholders::_1));

    if (!openSerial())
    {
      throw std::runtime_error("Failed to open serial port");
    }

    if (send_zero_cmd_on_startup_)
    {
      sendVelocityFrame(0.0f, 0.0f);
      RCLCPP_INFO(get_logger(), "Startup handshake sent: v=0.0, w=0.0");
    }

    last_cmd_time_ = now();
    last_publish_time_ = now();
    reader_thread_ = std::thread(&BaseDriverNode::serialReadLoop, this);

    const auto publish_period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_)));
    publish_timer_ = create_wall_timer(publish_period, std::bind(&BaseDriverNode::onPublishTimer, this));

    watchdog_timer_ = create_wall_timer(100ms, std::bind(&BaseDriverNode::onWatchdogTimer, this));

    if (zero_cmd_hz_ > 1e-6)
    {
      const auto zero_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / zero_cmd_hz_));
      zero_cmd_timer_ = create_wall_timer(zero_period, std::bind(&BaseDriverNode::onZeroCmdTimer, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "base_driver_node started on %s @ %d (yaw_delta_scale=%.4f wz_scale=%.4f vx_scale=%.4f)",
      port_.c_str(), baudrate_, imu_yaw_delta_scale_, imu_wz_scale_, vx_scale_);
  }

  ~BaseDriverNode() override
  {
    running_.store(false);
    if (reader_thread_.joinable())
    {
      reader_thread_.join();
    }
    if (serial_fd_ >= 0)
    {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  enum class ParserState
  {
    kWaitSof,
    kWaitCmd,
    kReadPayload,
    kWaitEof
  };

  struct FeedbackState
  {
    double vx{0.0};
    double vy{0.0};
    double yaw{0.0};
    double wz{0.0};
  };

  static speed_t toPosixBaud(const int baudrate)
  {
    switch (baudrate)
    {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      default: return B115200;
    }
  }

  bool openSerial()
  {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }

    cfmakeraw(&tty);
    const auto baud = toPosixBaud(baudrate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return false;
    }

    int flags = fcntl(serial_fd_, F_GETFL, 0);
    if (flags >= 0)
    {
      fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK);
    }

    return true;
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const float target_v = static_cast<float>(msg->linear.x);
    const float target_w = static_cast<float>(msg->angular.z);
    sendVelocityFrame(target_v, target_w);
    last_cmd_time_ = now();
  }

  void onZeroCmdTimer()
  {
    if (!send_zero_cmd_on_startup_)
    {
      return;
    }
    const auto dt = (now() - last_cmd_time_).seconds();
    if (dt > cmd_watchdog_timeout_s_)
    {
      sendVelocityFrame(0.0f, 0.0f);
    }
  }

  void onWatchdogTimer()
  {
    const auto dt = (now() - last_cmd_time_).seconds();
    if (dt > cmd_watchdog_timeout_s_)
    {
      sendVelocityFrame(0.0f, 0.0f);
      last_cmd_time_ = now();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cmd_vel timeout %.2fs, sending stop frame", dt);
    }
  }

  void sendVelocityFrame(const float target_v, const float target_w)
  {
    if (serial_fd_ < 0)
    {
      return;
    }
    std::array<uint8_t, kFrameSize> frame{};
    frame[0] = kSof;
    frame[1] = kCmdDownstream;
    std::memcpy(frame.data() + 2, &target_v, sizeof(float));
    std::memcpy(frame.data() + 6, &target_w, sizeof(float));
    frame[10] = kEof;

    const auto ret = write(serial_fd_, frame.data(), frame.size());
    if (ret < 0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "serial write failed: %s", std::strerror(errno));
    }
  }

  void serialReadLoop()
  {
    while (running_.load())
    {
      uint8_t byte = 0;
      const auto n = read(serial_fd_, &byte, 1);
      if (n <= 0)
      {
        std::this_thread::sleep_for(1ms);
        continue;
      }
      parseByte(byte);
    }
  }

  void parseByte(const uint8_t byte)
  {
    switch (state_)
    {
      case ParserState::kWaitSof:
        if (byte == kSof)
        {
          state_ = ParserState::kWaitCmd;
        }
        break;
      case ParserState::kWaitCmd:
        if (byte == kCmdVelFeedback || byte == kCmdYawFeedback)
        {
          current_cmd_ = byte;
          payload_index_ = 0;
          state_ = ParserState::kReadPayload;
        }
        else
        {
          state_ = ParserState::kWaitSof;
        }
        break;
      case ParserState::kReadPayload:
        payload_[payload_index_++] = byte;
        if (payload_index_ >= kPayloadSize)
        {
          state_ = ParserState::kWaitEof;
        }
        break;
      case ParserState::kWaitEof:
        if (byte == kEof)
        {
          onValidFrame(current_cmd_, payload_);
        }
        state_ = ParserState::kWaitSof;
        break;
      default:
        state_ = ParserState::kWaitSof;
        break;
    }
  }

  static float bytesToFloat(const uint8_t * data)
  {
    float out{};
    std::memcpy(&out, data, sizeof(float));
    return out;
  }

  void onValidFrame(const uint8_t cmd, const std::array<uint8_t, kPayloadSize> & payload)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (cmd == kCmdVelFeedback)
    {
      feedback_state_.vx = static_cast<double>(bytesToFloat(payload.data()));
      feedback_state_.vy = static_cast<double>(bytesToFloat(payload.data() + 4));
    }
    else if (cmd == kCmdYawFeedback)
    {
      const double raw_yaw = static_cast<double>(bytesToFloat(payload.data()));
      if (use_first_yaw_as_zero_ && !yaw_zero_initialized_)
      {
        yaw_zero_bias_ = raw_yaw;
        yaw_zero_initialized_ = true;
        RCLCPP_INFO(
          get_logger(), "Initialized yaw zero bias from first IMU frame: %.4f rad", yaw_zero_bias_);
      }
      const double zeroed_yaw = raw_yaw - yaw_zero_bias_;
      feedback_state_.yaw = normalizeAngle(zeroed_yaw + yaw_offset_rad_);
      feedback_state_.wz = static_cast<double>(bytesToFloat(payload.data() + 4));
    }

    feedback_stamp_ = now();

    if (enable_rx_log_)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "RX feedback: vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s, yaw=%.3f rad",
        feedback_state_.vx, feedback_state_.vy, feedback_state_.wz, feedback_state_.yaw);
    }
  }

  void fillCovariance(std::array<double, 36> & cov, const std::vector<double> & diag)
  {
    cov.fill(0.0);
    for (std::size_t i = 0; i < 6 && i < diag.size(); ++i)
    {
      cov[i * 6 + i] = diag[i];
    }
  }

  void onPublishTimer()
  {
    const auto stamp = now();
    const auto dt = (stamp - last_publish_time_).seconds();
    last_publish_time_ = stamp;
    if (dt <= 0.0)
    {
      return;
    }

    FeedbackState snapshot;
    rclcpp::Time feedback_stamp;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      snapshot = feedback_state_;
      feedback_stamp = feedback_stamp_;
    }

    const auto feedback_age_s = (stamp - feedback_stamp).seconds();
    if (feedback_age_s > feedback_timeout_s_)
    {
      // Avoid integrating stale velocity frames when serial feedback stalls.
      snapshot.vx = 0.0;
      snapshot.vy = 0.0;
      snapshot.wz = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "stale feedback (age=%.3fs > %.3fs), zeroing twist for odom integration",
        feedback_age_s, feedback_timeout_s_);
    }

    if (std::fabs(snapshot.vx) < vx_deadband_mps_)
    {
      snapshot.vx = 0.0;
    }
    if (std::fabs(snapshot.vy) < vy_deadband_mps_)
    {
      snapshot.vy = 0.0;
    }
    if (std::fabs(snapshot.wz) < wz_deadband_rps_)
    {
      snapshot.wz = 0.0;
    }
    if (assume_nonholonomic_)
    {
      // Differential-drive chassis should not integrate lateral velocity.
      snapshot.vy = 0.0;
    }

    snapshot.vx *= vx_scale_;
    snapshot.wz *= imu_wz_scale_;

    if (!mcu_yaw_initialized_)
    {
      last_mcu_yaw_ = snapshot.yaw;
      corrected_yaw_ = snapshot.yaw;
      mcu_yaw_initialized_ = true;
    }
    else
    {
      const double dyaw = normalizeAngle(snapshot.yaw - last_mcu_yaw_);
      corrected_yaw_ = normalizeAngle(corrected_yaw_ + dyaw * imu_yaw_delta_scale_);
      last_mcu_yaw_ = snapshot.yaw;
    }
    snapshot.yaw = corrected_yaw_;

    const double c = std::cos(snapshot.yaw);
    const double s = std::sin(snapshot.yaw);
    x_ += (snapshot.vx * c - snapshot.vy * s) * dt;
    y_ += (snapshot.vx * s + snapshot.vy * c) * dt;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, snapshot.yaw);
    q.normalize();

    nav_msgs::msg::Odometry odom{};
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = snapshot.vx;
    odom.twist.twist.linear.y = snapshot.vy;
    odom.twist.twist.angular.z = snapshot.wz;
    fillCovariance(odom.pose.covariance, pose_cov_diag_);
    fillCovariance(odom.twist.covariance, twist_cov_diag_);
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu{};
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_id_;
    imu.orientation = tf2::toMsg(q);
    imu.angular_velocity.z = snapshot.wz;
    imu.orientation_covariance[0] = 1e6;
    imu.orientation_covariance[4] = 1e6;
    imu.orientation_covariance[8] = 0.05;
    imu.angular_velocity_covariance[0] = 1e6;
    imu.angular_velocity_covariance[4] = 1e6;
    imu.angular_velocity_covariance[8] = 0.05;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped tf_msg{};
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = base_frame_id_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf_msg);
  }

  // Parameters
  std::string port_;
  int baudrate_{115200};
  double publish_rate_hz_{50.0};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_footprint"};
  std::string imu_frame_id_{"imu_link"};
  double yaw_offset_rad_{0.0};
  bool use_first_yaw_as_zero_{true};
  bool assume_nonholonomic_{true};
  double vx_deadband_mps_{0.005};
  double vy_deadband_mps_{0.01};
  double wz_deadband_rps_{0.01};
  double feedback_timeout_s_{0.20};
  double imu_yaw_delta_scale_{360.0 / 478.0};
  double imu_wz_scale_{360.0 / 716.0};
  double vx_scale_{1.0};
  double cmd_watchdog_timeout_s_{0.5};
  bool send_zero_cmd_on_startup_{true};
  double zero_cmd_hz_{2.0};
  bool enable_rx_log_{true};
  std::vector<double> pose_cov_diag_;
  std::vector<double> twist_cov_diag_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr zero_cmd_timer_;

  // Serial + parser
  int serial_fd_;
  std::atomic<bool> running_;
  std::thread reader_thread_;
  ParserState state_;
  uint8_t current_cmd_{0};
  std::array<uint8_t, kPayloadSize> payload_{};
  std::size_t payload_index_;

  // Feedback state
  std::mutex state_mutex_;
  FeedbackState feedback_state_;
  bool yaw_zero_initialized_{false};
  double yaw_zero_bias_{0.0};
  rclcpp::Time feedback_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  // Integrated pose
  double x_{0.0};
  double y_{0.0};
  bool mcu_yaw_initialized_{false};
  double last_mcu_yaw_{0.0};
  double corrected_yaw_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<BaseDriverNode>());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Fatal in base_driver_node: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

