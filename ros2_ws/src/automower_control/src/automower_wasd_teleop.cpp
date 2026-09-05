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

  auto node = rclcpp::Node::make_shared("automower_wasd_teleop");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10);
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
  auto engineEnabledPublisher = node->create_publisher<std_msgs::msg::Bool>(
      "/engine_enabled",
      10);
  auto simulateLidarObstaclePublisher = node->create_publisher<std_msgs::msg::Bool>(
      "/simulate_lidar_obstacle",
      10);

  TerminalRawMode rawMode;

  std::cout
      << "WASD teleop\n"
      << "w/s: increase/decrease forward speed\n"
      << "a/d: increase/decrease turn rate\n"
      << "1: manual drive mode\n"
      << "2: mowing mode\n"
      << "b: blade profile\n"
      << "i: blade drive on-off\n"
      << "t: blade on-off\n"
      << "r/f: blade power up-down\n"
      << "g/h: lower-raise cut height input\n"
      << "o: simulate lidar obstacle\n"
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
  workMode.data = "mowing";

  std_msgs::msg::String toolProfile;
  toolProfile.data = "blade";

  std_msgs::msg::Bool toolEnabled;
  toolEnabled.data = false;

  std_msgs::msg::Bool engineEnabled;
  engineEnabled.data = false;

  std_msgs::msg::Bool emergencyStop;
  emergencyStop.data = false;

  std_msgs::msg::Float32 toolPower;
  toolPower.data = 0.0F;

  std_msgs::msg::Float32 toolAngle;
  toolAngle.data = 0.0F;

  std_msgs::msg::Bool simulateLidarObstacle;
  simulateLidarObstacle.data = false;

  rclcpp::WallRate loopRate(20.0);

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
        << " engine=" << (engineEnabled.data ? "1" : "0")
        << " power=" << toolPower.data
        << " blade_on=" << (toolEnabled.data ? "1" : "0")
        << " lidar=" << (simulateLidarObstacle.data ? "1" : "0")
        << " tool_angle=" << toolAngle.data
        << " estop=" << (emergencyStop.data ? "1" : "0")
        << "        "
        << std::flush;
  };

  auto publishOperatorState = [&]()
  {
    workModePublisher->publish(workMode);
    emergencyStopPublisher->publish(emergencyStop);
    toolProfilePublisher->publish(toolProfile);
    toolEnabledPublisher->publish(toolEnabled);
    toolPowerPublisher->publish(toolPower);
    toolAnglePublisher->publish(toolAngle);
    engineEnabledPublisher->publish(engineEnabled);
    simulateLidarObstaclePublisher->publish(simulateLidarObstacle);
  };

  printStatus(currentCmd);
  publishOperatorState();

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
        workMode.data = "manual_drive";
        printStatus(currentCmd);
        break;

      case '2':
        workMode.data = "mowing";
        toolProfile.data = "blade";
        printStatus(currentCmd);
        break;

      case 'b':
      case 'B':
        toolProfile.data = "blade";
        printStatus(currentCmd);
        break;

      case 't':
      case 'T':
        toolEnabled.data = !toolEnabled.data;
        printStatus(currentCmd);
        break;

      case 'i':
      case 'I':
        engineEnabled.data = !engineEnabled.data;
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

      case 'o':
      case 'O':
        simulateLidarObstacle.data = !simulateLidarObstacle.data;
        printStatus(currentCmd);
        break;

      case 'e':
      case 'E':
        emergencyStop.data = !emergencyStop.data;
        if (emergencyStop.data)
        {
          currentCmd = stopCmd;
          engineEnabled.data = false;
          toolEnabled.data = false;
          toolPower.data = 0.0F;
          toolAngle.data = 0.0F;
          simulateLidarObstacle.data = false;
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
        currentCmd = stopCmd;
        engineEnabled.data = false;
        toolEnabled.data = false;
        toolPower.data = 0.0F;
        simulateLidarObstacle.data = false;
        publisher->publish(stopCmd);
        publishOperatorState();
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