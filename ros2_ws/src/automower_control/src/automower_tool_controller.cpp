#include <algorithm>
#include <array>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class AutomowerToolController : public rclcpp::Node
{
public:
  AutomowerToolController()
      : Node("automower_tool_controller")
  {
    profileSubscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "/tool_profile",
            10,
            std::bind(
                &AutomowerToolController::profileCallback,
                this,
                std::placeholders::_1));

    enabledSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/tool_enabled",
            10,
            std::bind(
                &AutomowerToolController::enabledCallback,
                this,
                std::placeholders::_1));

    powerSubscription_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/tool_power",
            10,
            std::bind(
                &AutomowerToolController::powerCallback,
                this,
                std::placeholders::_1));

    angleSubscription_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/tool_angle",
            10,
            std::bind(
                &AutomowerToolController::angleCallback,
                this,
                std::placeholders::_1));

    workModeSubscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "/work_mode",
            10,
            std::bind(
                &AutomowerToolController::workModeCallback,
                this,
                std::placeholders::_1));

    emergencyStopSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop",
            10,
            std::bind(
                &AutomowerToolController::emergencyStopCallback,
                this,
                std::placeholders::_1));

    engineEnabledSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/engine_enabled",
            10,
            std::bind(
                &AutomowerToolController::engineEnabledCallback,
                this,
                std::placeholders::_1));

    lidarObstacleSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/simulate_lidar_obstacle",
            10,
            std::bind(
                &AutomowerToolController::lidarObstacleCallback,
                this,
                std::placeholders::_1));

    bladeEnabledPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/mower/blade_enabled",
            10);

    bladeRpmPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/mower/blade_rpm",
            10);

    cutHeightPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/mower/cut_height",
            10);

    lidarObstaclePublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/mower/lidar_obstacle",
            10);

    mowerSafetyStopPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/mower/safety_stop",
            10);

    mowerStatusTextPublisher_ =
        this->create_publisher<std_msgs::msg::String>(
            "/mower/status_text",
            10);

    lidarScanPublisher_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan",
            10);

    statusTimer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(
                &AutomowerToolController::updateMowerState,
                this));

    RCLCPP_INFO(
        this->get_logger(),
        "tool controller ready in mode '%s' with profile '%s'",
        workMode_.c_str(),
        toolProfile_.c_str());
  }

private:
  static constexpr float BLADE_MAX_RPM = 3200.0F;
  static constexpr float DEFAULT_BLADE_IDLE_POWER = 0.55F;
  static constexpr float CUT_HEIGHT_MIN = 0.06F;
  static constexpr float CUT_HEIGHT_MAX = 0.12F;
  static constexpr double OPERATOR_COMMAND_TIMEOUT = 0.4;
  static constexpr std::size_t LIDAR_SAMPLE_COUNT = 181;
  static constexpr float LIDAR_MIN_RANGE = 0.12F;
  static constexpr float LIDAR_MAX_RANGE = 6.0F;
  static constexpr float LIDAR_CLEAR_RANGE = 4.5F;
  static constexpr float LIDAR_OBSTACLE_RANGE = 0.65F;

  void profileCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    markOperatorCommand();

    const std::string nextProfile = normalizeProfile(msg->data);

    if (nextProfile == toolProfile_)
    {
      return;
    }

    toolProfile_ = nextProfile;

    RCLCPP_INFO(
        this->get_logger(),
        "tool profile set to '%s'",
        toolProfile_.c_str());
  }

  void enabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    markOperatorCommand();
    toolEnabledCommand_ = msg->data;
  }

  void powerCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    markOperatorCommand();
    toolPowerCommand_ = std::clamp(msg->data, 0.0F, 1.0F);
  }

  void angleCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    markOperatorCommand();
    toolAngleCommand_ = std::clamp(msg->data, -1.0F, 1.0F);
  }

  void workModeCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    markOperatorCommand();
    workMode_ = msg->data;
  }

  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    markOperatorCommand();
    emergencyStopActive_ = msg->data;

    if (emergencyStopActive_)
    {
      engineEnabledCommand_ = false;
      toolEnabledCommand_ = false;
    }
  }

  void engineEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    markOperatorCommand();
    engineEnabledCommand_ = msg->data;
  }

  void markOperatorCommand()
  {
    hasLastOperatorCommandTime_ = true;
    lastOperatorCommandTime_ = this->now();
  }

  void lidarObstacleCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    simulateLidarObstacle_ = msg->data;
  }

  void updateMowerState()
  {
    const auto now = this->now();
    const bool operatorCommandFresh =
        hasLastOperatorCommandTime_ &&
        (now - lastOperatorCommandTime_).seconds() < OPERATOR_COMMAND_TIMEOUT;

    const bool bladeModeActive =
        !emergencyStopActive_ &&
        workMode_ == "mowing" &&
        toolProfile_ == "blade";

    const bool bladeDriveActive =
        operatorCommandFresh &&
        engineEnabledCommand_ &&
        !emergencyStopActive_;

    const bool effectiveBladeEnabled =
        bladeModeActive &&
        bladeDriveActive &&
        toolEnabledCommand_ &&
        !simulateLidarObstacle_;

    const float bladePowerDemand =
        std::max(toolPowerCommand_, DEFAULT_BLADE_IDLE_POWER);

    const float bladeRpm =
        effectiveBladeEnabled
            ? bladePowerDemand * BLADE_MAX_RPM
            : 0.0F;

    const float cutHeight =
        std::clamp(
            CUT_HEIGHT_MIN +
                ((toolAngleCommand_ + 1.0F) * 0.5F) *
                    (CUT_HEIGHT_MAX - CUT_HEIGHT_MIN),
            CUT_HEIGHT_MIN,
            CUT_HEIGHT_MAX);

    const bool mowerSafetyStop =
        emergencyStopActive_ || simulateLidarObstacle_;

    const std::string mowerStatusText =
        buildMowerStatusText(
            operatorCommandFresh,
            bladeModeActive,
            bladeDriveActive,
            effectiveBladeEnabled,
            mowerSafetyStop,
            toolEnabledCommand_);

    publishMowerTopics(
        effectiveBladeEnabled,
        bladeRpm,
        cutHeight,
        mowerSafetyStop,
        mowerStatusText);

    publishLidarScan(now);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "mode=%s profile=%s engine=%s tool=%s power=%.2f cut_height=%.2f blade=%s blade_rpm=%.0f obstacle=%s safety_stop=%s status=%s",
        workMode_.c_str(),
        toolProfile_.c_str(),
        bladeDriveActive ? "on" : "off",
        toolEnabledCommand_ ? "on" : "off",
        toolPowerCommand_,
        cutHeight,
        effectiveBladeEnabled ? "on" : "off",
        bladeRpm,
        simulateLidarObstacle_ ? "true" : "false",
        mowerSafetyStop ? "true" : "false",
        mowerStatusText.c_str());
  }

  void publishMowerTopics(
      bool effectiveBladeEnabled,
      float bladeRpm,
      float cutHeight,
      bool mowerSafetyStop,
      const std::string &mowerStatusText)
  {
    std_msgs::msg::Bool bladeEnabledMsg;
    bladeEnabledMsg.data = effectiveBladeEnabled;
    bladeEnabledPublisher_->publish(bladeEnabledMsg);

    std_msgs::msg::Float32 bladeRpmMsg;
    bladeRpmMsg.data = bladeRpm;
    bladeRpmPublisher_->publish(bladeRpmMsg);

    std_msgs::msg::Float32 cutHeightMsg;
    cutHeightMsg.data = cutHeight;
    cutHeightPublisher_->publish(cutHeightMsg);

    std_msgs::msg::Bool lidarObstacleMsg;
    lidarObstacleMsg.data = simulateLidarObstacle_;
    lidarObstaclePublisher_->publish(lidarObstacleMsg);

    std_msgs::msg::Bool mowerSafetyStopMsg;
    mowerSafetyStopMsg.data = mowerSafetyStop;
    mowerSafetyStopPublisher_->publish(mowerSafetyStopMsg);

    std_msgs::msg::String mowerStatusTextMsg;
    mowerStatusTextMsg.data = mowerStatusText;
    mowerStatusTextPublisher_->publish(mowerStatusTextMsg);
  }

  void publishLidarScan(const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::LaserScan scan;
    const float angleMin = -1.5707963F;
    const float angleMax = 1.5707963F;
    const float angleIncrement =
        (angleMax - angleMin) /
        static_cast<float>(LIDAR_SAMPLE_COUNT - 1U);

    scan.header.stamp = stamp;
    scan.header.frame_id = "lidar_link";
    scan.angle_min = angleMin;
    scan.angle_max = angleMax;
    scan.angle_increment = angleIncrement;
    scan.time_increment = 0.0F;
    scan.scan_time = 0.2F;
    scan.range_min = LIDAR_MIN_RANGE;
    scan.range_max = LIDAR_MAX_RANGE;
    scan.ranges.assign(LIDAR_SAMPLE_COUNT, LIDAR_CLEAR_RANGE);
    scan.intensities.assign(LIDAR_SAMPLE_COUNT, 20.0F);

    if (simulateLidarObstacle_)
    {
      constexpr std::array<std::size_t, 9> obstacleIndices = {
          86U, 87U, 88U, 89U, 90U, 91U, 92U, 93U, 94U};

      for (const std::size_t index : obstacleIndices)
      {
        scan.ranges[index] = LIDAR_OBSTACLE_RANGE;
        scan.intensities[index] = 180.0F;
      }
    }

    lidarScanPublisher_->publish(scan);
  }

  std::string buildMowerStatusText(
      bool operatorCommandFresh,
      bool bladeModeActive,
      bool bladeDriveActive,
      bool effectiveBladeEnabled,
      bool mowerSafetyStop,
      bool toolRequestActive) const
  {
    if (emergencyStopActive_)
    {
      return "emergency_stop_active";
    }

    if (!operatorCommandFresh)
    {
      return "operator_command_timeout";
    }

    if (!bladeModeActive)
    {
      return "switch_to_mowing_blade_profile";
    }

    if (simulateLidarObstacle_)
    {
      return "lidar_obstacle_stop";
    }

    if (!bladeDriveActive && toolRequestActive)
    {
      return "start_blade_drive";
    }

    if (mowerSafetyStop)
    {
      return "mower_safety_stop";
    }

    if (effectiveBladeEnabled)
    {
      return "blade_spinning";
    }

    if (toolRequestActive)
    {
      return "blade_requested_waiting";
    }

    if (!bladeDriveActive)
    {
      return "blade_drive_off";
    }

    return "ready_to_mow";
  }

  std::string normalizeProfile(const std::string &value) const
  {
    if (value.empty())
    {
      return "blade";
    }

    return value;
  }

  std::string toolProfile_ = "blade";
  std::string workMode_ = "mowing";

  bool engineEnabledCommand_ = false;
  bool toolEnabledCommand_ = false;
  bool emergencyStopActive_ = false;
  bool simulateLidarObstacle_ = false;
  bool hasLastOperatorCommandTime_ = false;

  float toolPowerCommand_ = 0.0F;
  float toolAngleCommand_ = 0.0F;

  rclcpp::Time lastOperatorCommandTime_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      profileSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      enabledSubscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      powerSubscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      angleSubscription_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      workModeSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      emergencyStopSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      engineEnabledSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      lidarObstacleSubscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      bladeEnabledPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      bladeRpmPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      cutHeightPublisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      lidarObstaclePublisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      mowerSafetyStopPublisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      mowerStatusTextPublisher_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      lidarScanPublisher_;

  rclcpp::TimerBase::SharedPtr statusTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutomowerToolController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}