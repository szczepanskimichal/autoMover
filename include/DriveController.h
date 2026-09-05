#pragma once

// Normalized wheel command set shared by the prototype app and ROS 2 node.
struct WheelSpeeds
{
    double frontLeft;
    double frontRight;
    double rearLeft;
    double rearRight;
};

class DriveController
{
public:
    // Mix throttle and steering into left/right wheel commands.
    WheelSpeeds calculate(double throttle, double steering) const;
};