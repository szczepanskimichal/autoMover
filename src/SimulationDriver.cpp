#include "SimulationDriver.h"

#include <iostream>

void SimulationDriver::apply(const WheelSpeeds& speeds) const
{
    std::cout
        << "FL: " << speeds.frontLeft
        << " | FR: " << speeds.frontRight
        << " | RL: " << speeds.rearLeft
        << " | RR: " << speeds.rearRight
        << '\n';
}