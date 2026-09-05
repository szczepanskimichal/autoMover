#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <limits>

using namespace webots;

int main()
{
    Robot robot;

    const int timeStep =
        static_cast<int>(robot.getBasicTimeStep());

    Motor* frontLeft =
        robot.getMotor("front_left_motor");

    Motor* frontRight =
        robot.getMotor("front_right_motor");

    Motor* rearLeft =
        robot.getMotor("rear_left_motor");

    Motor* rearRight =
        robot.getMotor("rear_right_motor");

    // Infinite position switches each wheel motor into velocity-control mode.
    frontLeft->setPosition(
        std::numeric_limits<double>::infinity());

    frontRight->setPosition(
        std::numeric_limits<double>::infinity());

    rearLeft->setPosition(
        std::numeric_limits<double>::infinity());

    rearRight->setPosition(
        std::numeric_limits<double>::infinity());

    frontLeft->setVelocity(2.0);
    rearLeft->setVelocity(2.0);

    frontRight->setVelocity(2.0);
    rearRight->setVelocity(2.0);

    while (robot.step(timeStep) != -1)
    {
    }

    return 0;
}