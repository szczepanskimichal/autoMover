#include <chrono>
#include <cstdio>
#include <iostream>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

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

  TerminalRawMode rawMode;

  std::cout
      << "WASD teleop\n"
      << "w/s: increase/decrease forward speed\n"
      << "a/d: increase/decrease turn rate\n"
      << "x or space: stop\n"
      << "q: quit\n";

  constexpr double linearStep = 0.12;
  constexpr double angularStep = 0.2;
  constexpr double maxLinearSpeed = 0.35;
  constexpr double maxAngularSpeed = 0.5;

  geometry_msgs::msg::Twist currentCmd;
  geometry_msgs::msg::Twist stopCmd;
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
        << "        "
        << std::flush;
  };

  printStatus(currentCmd);

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

      case 'x':
      case 'X':
      case ' ':
        currentCmd = stopCmd;
        printStatus(currentCmd);
        break;

      case 'q':
      case 'Q':
        publisher->publish(stopCmd);
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

    loopRate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}