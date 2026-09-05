#pragma once

#include "DriveController.h"

// Minimal abstraction for anything that can consume wheel commands.
class IRobotDriver
{
public:
    virtual ~IRobotDriver() = default;

    virtual void apply(const WheelSpeeds &speeds) = 0;
};