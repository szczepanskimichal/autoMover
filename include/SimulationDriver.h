#pragma once

#include "IRobotDriver.h"

// Console-backed driver used by the standalone prototype application.
class SimulationDriver : public IRobotDriver
{
public:
    void apply(const WheelSpeeds &speeds) override;
};