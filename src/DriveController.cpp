#include "DriveController.h"

#include <algorithm>

WheelSpeeds DriveController::calculate(double throttle, double steering) const
{
    // Steering biases the left and right side in opposite directions.
    double left = throttle + steering;
    double right = throttle - steering;

    left = std::clamp(left, -1.0, 1.0);
    right = std::clamp(right, -1.0, 1.0);

    WheelSpeeds speeds{};

    speeds.frontLeft = left;
    speeds.rearLeft = left;
    speeds.frontRight = right;
    speeds.rearRight = right;

    return speeds;
}