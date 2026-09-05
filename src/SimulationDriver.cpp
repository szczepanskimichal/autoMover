#include "SimulationDriver.h"

#include <iostream>

void SimulationDriver::apply(const WheelSpeeds &speeds)
{
    // The prototype driver only exposes the mixed wheel command values.
    std::cout
        << "FL: " << speeds.frontLeft
        << " | FR: " << speeds.frontRight
        << " | RL: " << speeds.rearLeft
        << " | RR: " << speeds.rearRight
        << '\n';
}