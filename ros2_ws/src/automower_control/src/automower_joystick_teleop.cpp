#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class AutomowerJoystickTeleop : public rclcpp::Node
{
public:
  AutomowerJoystickTeleop()
      : Node("automower_joystick_teleop"),
        hasJoy_(false)
  {
    axisLinear_ = this->declare_parameter<int>("axis_linear", 1);
    axisAngular_ = this->declare_parameter<int>("axis_angular", 0);
    enableButton_ = this->declare_parameter<int>("enable_button", -1);
    stopButton_ = this->declare_parameter<int>("stop_button", 0);
    scaleLinear_ = this->declare_parameter<double>("scale_linear", 0.35);
    scaleAngular_ = this->declare_parameter<double>("scale_angular", 0.6);
    deadzone_ = this->declare_parameter<double>("deadzone", 0.12);
    timeoutSeconds_ = this->declare_parameter<double>("timeout_seconds", 0.25);
    invertLinear_ = this->declare_parameter<bool>("invert_linear", true);
    invertAngular_ = this->declare_parameter<bool>("invert_angular", false);

    cmdPublisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

    joySubscription_ =
        this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(
                &AutomowerJoystickTeleop::joyCallback,
                this,
                std::placeholders::_1));

    timeoutTimer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &AutomowerJoystickTeleop::timeoutCallback,
                this));

    RCLCPP_INFO(
        this->get_logger(),
        "joystick teleop ready: axis_linear=%d axis_angular=%d enable_button=%d stop_button=%d",
        axisLinear_,
        axisAngular_,
        enableButton_,
        stopButton_);
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    hasJoy_ = true;
    lastJoyTime_ = this->now();

    if (isButtonPressed(msg, stopButton_))
    {
      publishStop();
      return;
    }

    if (enableButton_ >= 0 && !isButtonPressed(msg, enableButton_))
    {
      publishStop();
      return;
    }

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x =
        scaleLinear_ * readAxis(msg, axisLinear_, invertLinear_);

    cmd.angular.z =
        scaleAngular_ * readAxis(msg, axisAngular_, invertAngular_);

    lastPublishedCmd_ = cmd;
    cmdPublisher_->publish(cmd);
  }

  void timeoutCallback()
  {
    if (!hasJoy_)
    {
      return;
    }

    if ((this->now() - lastJoyTime_).seconds() > timeoutSeconds_)
    {
      publishStop();
    }
  }

  double readAxis(
      const sensor_msgs::msg::Joy::SharedPtr &msg,
      int axis,
      bool invert) const
  {
    if (axis < 0 || axis >= static_cast<int>(msg->axes.size()))
    {
      return 0.0;
    }

    double value = msg->axes[axis];

    if (invert)
    {
      value = -value;
    }

    if (std::abs(value) < deadzone_)
    {
      return 0.0;
    }

    return std::clamp(value, -1.0, 1.0);
  }

  bool isButtonPressed(
      const sensor_msgs::msg::Joy::SharedPtr &msg,
      int button) const
  {
    return button >= 0 &&
           button < static_cast<int>(msg->buttons.size()) &&
           msg->buttons[button] != 0;
  }

  void publishStop()
  {
    if (lastPublishedCmd_.linear.x == 0.0 &&
        lastPublishedCmd_.angular.z == 0.0)
    {
      return;
    }

    lastPublishedCmd_ = geometry_msgs::msg::Twist{};
    cmdPublisher_->publish(lastPublishedCmd_);
  }

  int axisLinear_;
  int axisAngular_;
  int enableButton_;
  int stopButton_;

  double scaleLinear_;
  double scaleAngular_;
  double deadzone_;
  double timeoutSeconds_;

  bool invertLinear_;
  bool invertAngular_;
  bool hasJoy_;

  rclcpp::Time lastJoyTime_;
  geometry_msgs::msg::Twist lastPublishedCmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      cmdPublisher_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      joySubscription_;

  rclcpp::TimerBase::SharedPtr
      timeoutTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutomowerJoystickTeleop>());
  rclcpp::shutdown();
  return 0;
}