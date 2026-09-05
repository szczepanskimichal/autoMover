#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "DriveController.h"
#include "SimulationDriver.h"

class TerminalRawMode
{
public:
    TerminalRawMode()
    {
        tcgetattr(STDIN_FILENO, &oldSettings);

        termios newSettings = oldSettings;
        newSettings.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
    }

    ~TerminalRawMode()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    }

private:
    termios oldSettings{};
};

int main()
{
    TerminalRawMode rawMode;

    DriveController controller;
    SimulationDriver driver;

    double throttle = 0.0;
    double steering = 0.0;

    std::cout << "autoMower REAL-TIME control\n";
    std::cout << "W/S = throttle\n";
    std::cout << "A/D = steering\n";
    std::cout << "X   = stop\n";
    std::cout << "Q   = quit\n\n";

    // This loop is the smallest end-to-end demo of the motion model.
    while (true)
    {
        char command;
        read(STDIN_FILENO, &command, 1);

        switch (command)
        {
            case 'w':
            case 'W':
                throttle += 0.1;
                break;

            case 's':
            case 'S':
                throttle -= 0.1;
                break;

            case 'a':
            case 'A':
                steering -= 0.1;
                break;

            case 'd':
            case 'D':
                steering += 0.1;
                break;

            case 'x':
            case 'X':
                throttle = 0.0;
                steering = 0.0;
                break;

            case 'q':
            case 'Q':
                std::cout << "\nStopping autoMower.\n";
                return 0;

            default:
                continue;
        }

        if (throttle > 1.0) throttle = 1.0;
        if (throttle < -1.0) throttle = -1.0;

        if (steering > 1.0) steering = 1.0;
        if (steering < -1.0) steering = -1.0;

        const WheelSpeeds wheels =
            controller.calculate(throttle, steering);

        std::cout
            << "\rThrottle: " << throttle
            << " Steering: " << steering
            << "     \n";

        driver.apply(wheels);
    }
}