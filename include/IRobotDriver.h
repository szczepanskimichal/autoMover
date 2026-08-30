#pragma once

#include "DriveController.h"

class IRobotDriver
{
public:
    virtual ~IRobotDriver() = default;

    virtual void apply(const WheelSpeeds& speeds) = 0;
};