#pragma once

#include "IRobotDriver.h"

class SimulationDriver : public IRobotDriver
{
public:
    void apply(const WheelSpeeds& speeds) override;
};