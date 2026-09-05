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
      << "w: forward\n"
      << "s: backward\n"
      << "a: turn left\n"
      << "d: turn right\n"
      << "hold key: keep moving\n"
      << "space: stop\n"
      << "q: quit\n";

  geometry_msgs::msg::Twist currentCmd;
  geometry_msgs::msg::Twist stopCmd;
  auto lastMotionInput = std::chrono::steady_clock::now();
  bool publishStopOnce = true;
  rclcpp::WallRate loopRate(20.0);

  // Keep publishing while a key is held, then send one clean stop on timeout.
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
        nextCmd.linear.x = 0.6;
        currentCmd = nextCmd;
        lastMotionInput = std::chrono::steady_clock::now();
        publishStopOnce = false;
        break;

      case 's':
      case 'S':
        nextCmd.linear.x = -0.6;
        currentCmd = nextCmd;
        lastMotionInput = std::chrono::steady_clock::now();
        publishStopOnce = false;
        break;

      case 'a':
      case 'A':
        nextCmd.angular.z = 0.8;
        currentCmd = nextCmd;
        lastMotionInput = std::chrono::steady_clock::now();
        publishStopOnce = false;
        break;

      case 'd':
      case 'D':
        nextCmd.angular.z = -0.8;
        currentCmd = nextCmd;
        lastMotionInput = std::chrono::steady_clock::now();
        publishStopOnce = false;
        break;

      case ' ':
        currentCmd = stopCmd;
        publisher->publish(currentCmd);
        publishStopOnce = true;
        break;

      case 'q':
      case 'Q':
        publisher->publish(stopCmd);
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

    const auto now = std::chrono::steady_clock::now();
    const bool motionTimedOut =
        now - lastMotionInput > 250ms;

    if (motionTimedOut)
    {
      if (!publishStopOnce)
      {
        currentCmd = stopCmd;
        publisher->publish(currentCmd);
        publishStopOnce = true;
      }
    }
    else
    {
      publisher->publish(currentCmd);
    }

    loopRate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}