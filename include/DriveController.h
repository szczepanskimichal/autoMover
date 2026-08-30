#pragma once

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
    WheelSpeeds calculate(double throttle, double steering) const;
};