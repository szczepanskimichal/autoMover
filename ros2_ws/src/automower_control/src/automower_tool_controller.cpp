#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"
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

    statusTimer_ =
        this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(
                &AutomowerToolController::publishStatus,
                this));

    RCLCPP_INFO(
        this->get_logger(),
        "tool controller ready with profile '%s'",
        toolProfile_.c_str());
  }

private:
  void profileCallback(const std_msgs::msg::String::SharedPtr msg)
  {
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
    toolEnabled_ = msg->data;
  }

  void powerCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    toolPower_ = std::clamp(msg->data, 0.0F, 1.0F);
  }

  void angleCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    toolAngle_ = std::clamp(msg->data, -1.0F, 1.0F);
  }

  void workModeCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    workMode_ = msg->data;
  }

  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    emergencyStopActive_ = msg->data;

    if (emergencyStopActive_)
    {
      toolEnabled_ = false;
      toolPower_ = 0.0F;
    }
  }

  void publishStatus()
  {
    const bool toolAllowed =
        !emergencyStopActive_ &&
        ((toolProfile_ == "blade" && workMode_ == "mowing") ||
         (toolProfile_ == "auger" && workMode_ == "snow_clearing"));

    const bool effectiveEnabled = toolAllowed && toolEnabled_;
    const float effectivePower = effectiveEnabled ? toolPower_ : 0.0F;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "tool profile=%s mode=%s enabled=%s power=%.2f angle=%.2f effective_enabled=%s effective_power=%.2f",
        toolProfile_.c_str(),
        workMode_.c_str(),
        toolEnabled_ ? "true" : "false",
        toolPower_,
        toolAngle_,
        effectiveEnabled ? "true" : "false",
        effectivePower);
  }

  std::string normalizeProfile(const std::string &value) const
  {
    if (value == "blade" || value == "auger")
    {
      return value;
    }

    return "auger";
  }

  std::string toolProfile_ = "auger";
  std::string workMode_ = "manual_drive";

  bool toolEnabled_ = false;
  bool emergencyStopActive_ = false;

  float toolPower_ = 0.0F;
  float toolAngle_ = 0.0F;

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

  rclcpp::TimerBase::SharedPtr
      statusTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutomowerToolController>());
  rclcpp::shutdown();
  return 0;
}