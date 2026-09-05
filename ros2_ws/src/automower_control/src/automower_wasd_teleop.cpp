#include <chrono>
#include <cstdio>
#include <iostream>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class TerminalRawMode
{
public:
  TerminalRawMode()
      : active_(false)
  {
    if (tcgetattr(STDIN_FILENO, &original_) == 0)
    {
      termios raw = original_;

      raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 1;
      raw.c_cc[VTIME] = 0;

      if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
      {
        active_ = true;
      }
    }
  }

  ~TerminalRawMode()
  {
    if (active_)
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

private:
  termios original_{};
  bool active_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  using namespace std::chrono_literals;

  auto node = rclcpp::Node::make_shared("automower_wasd_teleop");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10);
    // Keep operator-facing mode and tool control in the same keyboard loop so a
    // single teleop process can drive both the chassis and the winter attachment.
  auto workModePublisher = node->create_publisher<std_msgs::msg::String>(
      "/work_mode",
      10);
  auto emergencyStopPublisher = node->create_publisher<std_msgs::msg::Bool>(
      "/emergency_stop",
      10);
  auto toolProfilePublisher = node->create_publisher<std_msgs::msg::String>(
      "/tool_profile",
      10);
  auto toolEnabledPublisher = node->create_publisher<std_msgs::msg::Bool>(
      "/tool_enabled",
      10);
  auto toolPowerPublisher = node->create_publisher<std_msgs::msg::Float32>(
      "/tool_power",
      10);
  auto toolAnglePublisher = node->create_publisher<std_msgs::msg::Float32>(
      "/tool_angle",
      10);

  TerminalRawMode rawMode;

  std::cout
      << "WASD teleop\n"
      << "w/s: increase/decrease forward speed\n"
      << "a/d: increase/decrease turn rate\n"
      << "1: manual mode\n"
      << "2: mowing mode\n"
      << "3: snow mode\n"
      << "b: blade profile\n"
      << "n: auger profile\n"
      << "t: tool on/off\n"
      << "r/f: tool power up/down\n"
      << "g/h: tool angle left/right\n"
      << "e: emergency stop toggle\n"
      << "x or space: stop\n"
      << "q: quit\n";

  constexpr double linearStep = 0.12;
  constexpr double angularStep = 0.2;
  constexpr double maxLinearSpeed = 0.35;
  constexpr double maxAngularSpeed = 0.5;

  geometry_msgs::msg::Twist currentCmd;
  geometry_msgs::msg::Twist stopCmd;
  std_msgs::msg::String workMode;
  workMode.data = "manual_drive";

  std_msgs::msg::String toolProfile;
  toolProfile.data = "auger";

  std_msgs::msg::Bool toolEnabled;
  toolEnabled.data = false;

  std_msgs::msg::Bool emergencyStop;
  emergencyStop.data = false;

  std_msgs::msg::Float32 toolPower;
  toolPower.data = 0.0F;

  std_msgs::msg::Float32 toolAngle;
  toolAngle.data = 0.0F;

  rclcpp::WallRate loopRate(20.0);

  // Keep command tuning local and readable instead of spreading literals across
  // the key handling branches.
  auto clamp = [](double value, double limit)
  {
    if (value > limit)
    {
      return limit;
    }

    if (value < -limit)
    {
      return -limit;
    }

    return value;
  };

  auto printStatus = [&](const geometry_msgs::msg::Twist &cmd)
  {
    std::cout
        << "\rlinear.x=" << cmd.linear.x
        << " angular.z=" << cmd.angular.z
        << " mode=" << workMode.data
        << " tool=" << toolProfile.data
        << " tool_on=" << (toolEnabled.data ? "1" : "0")
        << " power=" << toolPower.data
        << " angle=" << toolAngle.data
        << " estop=" << (emergencyStop.data ? "1" : "0")
        << "        "
        << std::flush;
  };

  auto publishOperatorState = [&]()
  {
    // Publish every loop so late subscribers still converge to the current
    // operator state without needing a dedicated latched command layer.
    workModePublisher->publish(workMode);
    emergencyStopPublisher->publish(emergencyStop);
    toolProfilePublisher->publish(toolProfile);
    toolEnabledPublisher->publish(toolEnabled);
    toolPowerPublisher->publish(toolPower);
    toolAnglePublisher->publish(toolAngle);
  };

  printStatus(currentCmd);
  publishOperatorState();

  // Keep publishing the current target command until the operator changes it.
  while (rclcpp::ok())
  {
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(STDIN_FILENO, &readSet);

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;

    const int ready = select(
        STDIN_FILENO + 1,
        &readSet,
        nullptr,
        nullptr,
        &timeout);

    if (ready > 0 && FD_ISSET(STDIN_FILENO, &readSet))
    {
      char key = 0;

      if (read(STDIN_FILENO, &key, 1) <= 0)
      {
        continue;
      }

      geometry_msgs::msg::Twist nextCmd;
      bool recognizedKey = true;

      switch (key)
      {
      case 'w':
      case 'W':
        nextCmd = currentCmd;
        nextCmd.linear.x = clamp(
            currentCmd.linear.x + linearStep,
            maxLinearSpeed);
        currentCmd = nextCmd;
        printStatus(currentCmd);
        break;

      case 's':
      case 'S':
        nextCmd = currentCmd;
        nextCmd.linear.x = clamp(
            currentCmd.linear.x - linearStep,
            maxLinearSpeed);
        currentCmd = nextCmd;
        printStatus(currentCmd);
        break;

      case 'a':
      case 'A':
        nextCmd = currentCmd;
        nextCmd.angular.z = clamp(
            currentCmd.angular.z + angularStep,
            maxAngularSpeed);
        currentCmd = nextCmd;
        printStatus(currentCmd);
        break;

      case 'd':
      case 'D':
        nextCmd = currentCmd;
        nextCmd.angular.z = clamp(
            currentCmd.angular.z - angularStep,
            maxAngularSpeed);
        currentCmd = nextCmd;
        printStatus(currentCmd);
        break;

      case '1':
        // Manual mode leaves the chassis free while keeping the current tool
        // selection available for later arming.
        workMode.data = "manual_drive";
        printStatus(currentCmd);
        break;

      case '2':
        workMode.data = "mowing";
        printStatus(currentCmd);
        break;

      case '3':
        // Snow mode defaults back to the auger profile so winter sessions start
        // from the attachment we currently care about most.
        workMode.data = "snow_clearing";
        toolProfile.data = "auger";
        printStatus(currentCmd);
        break;

      case 'b':
      case 'B':
        toolProfile.data = "blade";
        printStatus(currentCmd);
        break;

      case 'n':
      case 'N':
        toolProfile.data = "auger";
        printStatus(currentCmd);
        break;

      case 't':
      case 'T':
        toolEnabled.data = !toolEnabled.data;
        printStatus(currentCmd);
        break;

      case 'r':
      case 'R':
        toolPower.data = static_cast<float>(clamp(toolPower.data + 0.1F, 1.0));
        printStatus(currentCmd);
        break;

      case 'f':
      case 'F':
        toolPower.data = static_cast<float>(clamp(toolPower.data - 0.1F, 1.0));
        printStatus(currentCmd);
        break;

      case 'g':
      case 'G':
        toolAngle.data = static_cast<float>(clamp(toolAngle.data - 0.1F, 1.0));
        printStatus(currentCmd);
        break;

      case 'h':
      case 'H':
        toolAngle.data = static_cast<float>(clamp(toolAngle.data + 0.1F, 1.0));
        printStatus(currentCmd);
        break;

      case 'e':
      case 'E':
        emergencyStop.data = !emergencyStop.data;
        if (emergencyStop.data)
        {
          // Emergency stop clears motion and tool output immediately, while the
          // latched mode selection remains visible to the rest of the stack.
          currentCmd = stopCmd;
          toolEnabled.data = false;
          toolPower.data = 0.0F;
        }
        printStatus(currentCmd);
        break;

      case 'x':
      case 'X':
      case ' ':
        currentCmd = stopCmd;
        printStatus(currentCmd);
        break;

      case 'q':
      case 'Q':
        publisher->publish(stopCmd);
        emergencyStopPublisher->publish(std_msgs::msg::Bool{});
        std::cout << "\n";
        rclcpp::shutdown();
        return 0;

      default:
        recognizedKey = false;
        break;
      }

      if (!recognizedKey)
      {
        continue;
      }
    }

    publisher->publish(currentCmd);
    publishOperatorState();

    loopRate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}