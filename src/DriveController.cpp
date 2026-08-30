#include "DriveController.h"
#include <algorithm>
#include <cmath>

WheelSpeeds DriveController::calculate(
    double throttle,
    double steering) const
{
    double left = throttle + steering;
    double right = throttle - steering;

    left = std::clamp(left, -1.0, 1.0);
    right = std::clamp(right, -1.0, 1.0);

    if (std::abs(left) < 0.000001) left = 0.0;
    if (std::abs(right) < 0.000001) right = 0.0;
    return {
        left,
        right,
        left,
        right
    };
}